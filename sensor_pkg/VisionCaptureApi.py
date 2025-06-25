#!/bin/python3
from email import header
from re import A
import socket
import threading
import time
import cv2
import numpy as np
import struct
import mmap
import json
import sys
import os
import math
import copy
import platform
import time

# IsEnable ROS image topic forwarding
isEnableRosTrans = False
is_use_ros1 = True
isLinux = False
if platform.system().lower() == "linux":
    isLinux = True
    try:
        ros_version = os.getenv("ROS_VERSION")
        print("current ros environment", os.getenv("ROS_DISTRO"))
        from logging import exception
        from typing import Any
        from xml.etree.ElementTree import tostring
        import yaml

        # Import other ROS libraries
        from std_msgs.msg import String
        import sensor_msgs.msg as sensor
        import std_msgs.msg as std_msg
        from cv_bridge import CvBridge

        if ros_version == "1":
            import rospy
        else:
            import rclpy
            from rclpy.node import Node
            from rclpy.clock import Clock
            from rclpy.duration import Duration
            from rclpy.qos import qos_profile_sensor_data
            is_use_ros1 = False

    except ImportError:
        print("Failed to load ROS libs")


class Queue:
    """patch"""

    def __init__(self):
        self.items = []

    def enqueue(self, item):
        self.items.insert(0, item)

    def dequeue(self):
        return self.items.pop()

    def is_empty(self):
        return self.items == []

    def size(self):
        return len(self.items)


# Note: This message will be sent to port 20005 of the specified remote computer
# struct RflyTimeStmp{
#     int checksum; //Checksum, value is 123456789
#     int copterID; //ID of the current aircraft
#     long long SysStartTime; //Timestamp at the start of simulation (in milliseconds, Greenwich Mean Time origin)
#     long long SysCurrentTime;//Current timestamp (in milliseconds, Greenwich Mean Time origin)
#     long long HeartCount; //Heartbeat packet counter
# } 2i3q
class RflyTimeStmp:
    def __init__(self):
        self.checksum = 1234567897
        self.copterID = 0
        self.SysStartTime = 0  # Timestamp of the computer when CopterSim starts simulation (in seconds)
        self.SysCurrentTime = 0  # Current timestamp of the computer running CopterSim (in seconds)
        self.HeartCount = 0

        # Timestamp processed by the Python side.
        self.isCopterSimOnPC = False
        # Note: If CopterSim and this Python script are on the same computer, the values of SysCurrentTime and time.time() should be very close (max delay 10ms)
        # This difference is used to determine if CopterSim and this Python script are on the same computer
        self.rosStartTimeStmp = 0  # ROS timestamp of this Python script's computer when CopterSim starts simulation
        # Note: If CopterSim and this Python script are on the same computer
        # Then pyStartTimeStmp <-- SysStartTime directly uses the simulation start time recorded by CopterSim
        # Then rosStartTimeStmp <-- pyStartTimeStmp + ROSTime - time.time(), meaning add the offset of ROS time relative to the local machine

        self.pyStartTimeStmp = 0

        # If CopterSim and this Python script are not on the same computer
        # Then pyStartTimeStmp <-- time.time() - SysCurrentTime + SysStartTime - 0.01
        # That is, deduce the CopterSim start time based on the current simulation time (SysCurrentTime-SysStartTime), with an added 10 millisecond delay allowance
        # Then rosStartTimeStmp <-- pyStartTimeStmp + ROSTime - time.time(), meaning add the offset of ROS time relative to the local machine

    def __init__(self, iv):
        self.checksum = iv[0]
        self.copterID = iv[1]
        self.SysStartTime = iv[2] / 1000.0
        self.SysCurrentTime = iv[3] / 1000.0
        self.HeartCount = iv[4]

    def Update(self, iv):
        self.checksum = iv[0]
        self.copterID = iv[1]
        self.SysStartTime = iv[2] / 1000.0
        self.SysCurrentTime = iv[3] / 1000.0
        self.HeartCount = iv[4]


class VisionSensorReq:
    """This is a class (C++ struct) that sent to UE4 to request and set camera parameters.
    # struct VisionSensorReq {
        uint16 checksum; //Data checksum, 12345
        uint16 SeqID; //Memory sequence ID
        uint16 TypeID; //Sensor type ID
        uint16 TargetCopter; //Bound target aircraft     //Changeable
        uint16 TargetMountType; //Binding type    //Changeable
        uint16 DataWidth;   //Data or image width
        uint16 DataHeight; //Data or image height
        uint16 DataCheckFreq; //Frequency for checking data updates
        uint16 SendProtocol[8]; //Transmission type (shared memory, UDP uncompressed, UDP video stream), IP address, port number, ...
        float CameraFOV;  //Camera field of view (visual sensors only)  //Changeable
        float SensorPosXYZ[3]; // Sensor installation position    //Changeable
        float EularOrQuat; //Select Euler angle or quaternion method, >0.5 means quaternion
        float SensorAngEular[3]; //Sensor installation angle   //Changeable
        float SensorAngQuat[4]; //Sensor installation quaternion   //Changeable
        float otherParams[16]; //Reserved 16 float data slots
    # }16H28f
    """

    def __init__(self):
        self.checksum = 12345
        self.SeqID = 0
        self.TypeID = 1
        self.TargetCopter = 1
        self.TargetMountType = 0
        self.DataWidth = 0
        self.DataHeight = 0
        self.DataCheckFreq = 0
        self.SendProtocol = [0] * 8
        self.CameraFOV = 90
        self.EularOrQuat = 0
        self.SensorAngQuat = [0, 0, 0, 0]
        self.SensorPosXYZ = [0, 0, 0]
        self.SensorAngEular = [0, 0, 0]
        self.otherParams = [0] * 16


class imuDataCopter:
    """This is a class (C++ struct) for IMU data receive from CopterSim
    # struct imuDataCopter{
    #     int checksum; //Data checksum 1234567898
    #     int seq; //Message sequence number
    #     double timestmp;//Timestamp
    #     float acc[3];
    #     float rate[3];
    # }   //2i1d6f
    """

    def __init__(self, imu_name="/rflysim/imu", node=None):
        global isEnableRosTrans
        global is_use_ros1
        self.checksum = 1234567898
        self.seq = 0
        self.timestmp = 0
        self.acc = [0, 0, 0]
        self.rate = [0, 0, 0]
        self.imuStmp = 0  # Corrected IMU timestamp
        self.rflyStartStmp = 0  # Timestamp when CopterSim started simulation
        if isEnableRosTrans:
            self.time_record = -1
            self.isUseTimeAlign = True  # Whether to publish data aligned with image time
            if len(imu_name) == 0:
                imu_name = "/rflysim/imu"
            if is_use_ros1:
                self.ns = rospy.get_namespace()
                if len(self.ns) > 1:
                    imu_name = self.ns + "rflysim/imu"
                self.imu_pub = rospy.Publisher(imu_name, sensor.Imu, queue_size=1)
                self.rostime = rospy.Time.now()

            else:
                self.rostime = node.get_clock().now()
                self.imu_pub = node.create_publisher(sensor.Imu, imu_name, 1)
            self.time_queue = Queue()
            self.newest_time_img = -1
            self.test_imu_time = 0
            self.test_sum = 0
            self.count = 0
            self.ros_imu = sensor.Imu()
            self.imu_frame_id = "imu"
            self.ros_imu.header.frame_id = self.imu_frame_id

    def AlignTime(self, img_time):  # Atomic operations do not require locks; using them reduces efficiency
        self.newest_time_img = img_time
        # print("queue size: ",self.time_queue.size())
        # print("current <img:%f,imu:%f>"% (img_time,self.test_imu_time))
        # self.test_sum += abs(self.newest_time_img - self.test_imu_time)
        # self.count +=1
        # if(self.count == 10):
        #     print("====",self.test_sum/self.count)
        #     self.count = 0
        #     self.test_sum = 0

        pass

    def Imu2ros(self, node=None):
        global is_use_ros1
        if isEnableRosTrans:
            # ros_imu = sensor.Imu()
            if is_use_ros1:
                self.ros_imu.header.stamp = rospy.Duration(self.imuStmp)
            else:
                rclpy_time = self.rostime + Duration(
                    seconds=self.imuStmp, nanoseconds=0
                )
                # self.ros_imu.header.stamp = rclpy_time.to_msg()
                seconds, nanoseconds = rclpy_time.seconds_nanoseconds()
                self.ros_imu.header.stamp.sec = int(seconds & 0x7FFFFFFF)
                self.ros_imu.header.stamp.nanosec = nanoseconds
            self.ros_imu.orientation.w = 0
            self.ros_imu.orientation.x = 0
            self.ros_imu.orientation.y = 0
            self.ros_imu.orientation.z = 0
            self.ros_imu.orientation_covariance[0] = -1
            self.ros_imu.orientation_covariance[1] = -1
            self.ros_imu.orientation_covariance[2] = -1
            self.ros_imu.orientation_covariance[3] = -1
            self.ros_imu.orientation_covariance[4] = -1
            self.ros_imu.orientation_covariance[5] = -1
            self.ros_imu.orientation_covariance[6] = -1
            self.ros_imu.orientation_covariance[7] = -1
            self.ros_imu.orientation_covariance[8] = -1
            self.ros_imu.linear_acceleration.x = self.acc[0]
            self.ros_imu.linear_acceleration.y = -self.acc[1]
            self.ros_imu.linear_acceleration.z = -self.acc[2]
            self.ros_imu.angular_velocity.x = self.rate[0]
            self.ros_imu.angular_velocity.y = -self.rate[1]
            self.ros_imu.angular_velocity.z = -self.rate[2]
            self.ros_imu.angular_velocity_covariance[0] = 0.001
            self.ros_imu.angular_velocity_covariance[4] = 0.001
            self.ros_imu.angular_velocity_covariance[8] = 0.001
            # self.ros_imu.header.stamp.secs = self.timestmp
            self.imu_pub.publish(self.ros_imu)


class DistanceSensor:
    def __init__(self):
        self.Distance = 0
        self.CopterID = 0
        self.RayStart = []
        self.AngEular = []
        self.ImpactPoint = []
        self.BoxOri = []


class SensorReqCopterSim:
    """This is a class (C++ struct) that sent to UE4 to request sensor data.
    # struct SensorReqCopterSim{
    #     uint16_t checksum;
    #     uint16_t sensorType;
    #     uint16_t updateFreq;
    #     uint16_t port;
    #     uint8_t IP[4];
    #     float Params[6];
    # } //4H4B6f
    """

    def __init__(self):
        self.checksum = 12345
        self.sensorType = 0
        self.updateFreq = 100
        self.port = 9998
        self.IP = [127, 0, 0, 1]
        self.Params = [0, 0, 0, 0, 0, 0]


class VisionCaptureApi:
    """This is the API class for python to request image from UE4"""

    def __del__(self):
        global isEnableRosTrans
        global is_use_ros1
        if isEnableRosTrans and not is_use_ros1:
            self.ros_node.destroy_node()
            rclpy.shutdown()

    def __init__(self, ip="127.0.0.1"):
        global isEnableRosTrans
        global is_use_ros1
        if isEnableRosTrans:
            if is_use_ros1:
                rospy.init_node("RecvRFlySim3DData", anonymous=True)
            else:
                rclpy.init()
                self.ros_node = Node("RecvRFlySim3DData")
                # print("If you want use to ROS2, please set rosnode_id at initialize  class VisionCaptureApi")
                # sys.exit(-1)
            self.time_record = Any
            self.rostime = Any
            # Add ROS node initialization work
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_imu.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.hostIp = socket.gethostbyname(socket.gethostname())  # Get local computer's IP
        self.VisSensor = []
        self.Img = []
        self.Img_lock = []  # Add thread lock for multi-threaded data synchronization
        self.ImgData = []
        self.hasData = []
        self.timeStmp = []
        self.imgStmp = []
        self.rflyStartStmp = []
        self.IpList = []
        self.portList = []
        self.hasReqUE4 = False
        self.sleepCheck = 0.005
        self.ip = ip
        self.isRemoteSend = False
        self.RemotSendIP = ""
        self.isUE4DirectUDP = False
        self.hasIMUData = False
        self.RflyTimeVect = []
        self.isNewJson = False
        self.tTimeStmpFlag = False
        self.DistanceSensor = 0
        self.startTime = time.time()
        self.isPrintTime = False
        self.lastIMUTime = time.time()
        self.sensors_num = 0
        if isEnableRosTrans:
            self.sensors_frame_id = ["map"]
            self.imu_frame_id = "imu"
            self.imu_topic_name = ""
            self.sensor_pub = {}
            self.sensor_data = {}
            self.cv_bridge = CvBridge()
            self.topic_name = {}
            try:
                file = open(r"tf_cfg.yaml")
                y = yaml.safe_load(file)
                self.imu_frame_id = y["imu_frame_id"]
                self.sensors_frame_id = y["sensors_frame_id"]
                self.sensors_num = y["sensors_num"]
                self.imu_topic_name = y["imu_topic_name"]
            except IOError:
                print("Using default global coordinate system frame_id: map")

        if isEnableRosTrans:
            if not is_use_ros1:
                self.imu = imuDataCopter(
                    imu_name=self.imu_topic_name, node=self.ros_node
                )
                self.imu.imu_frame_id = self.imu_frame_id
            else:
                self.imu = imuDataCopter(imu_name=self.imu_topic_name)
                self.imu.imu_frame_id = self.imu_frame_id
        else:
            self.imu = imuDataCopter()
            if isEnableRosTrans:
                self.imu.imu_frame_id = self.imu_frame_id
        # self.lock = threading.Lock() #Applied to IMU and image data time alignment processing

    def addVisSensor(self, vsr=VisionSensorReq()):
        """Add a new VisionSensorReq struct to the list"""
        if isinstance(vsr, VisionSensorReq):
            self.VisSensor = self.VisSensor + [copy.deepcopy(vsr)]
        else:
            raise Exception("Wrong data input to addVisSensor()")

    def sendReqToCopterSim(self, srcs=SensorReqCopterSim(), copterID=1,IP='127.0.0.1'):
        """send UDP message SensorReqCopterSim to CopterSim to request a sensor data
        the copterID specify the index of CopterSim to request
        """
        if type(srcs).__name__ != "SensorReqCopterSim":
            print("Error: input is not SensorReqCopterSim class")
            return
        u16Value = [srcs.checksum, srcs.sensorType, srcs.updateFreq, srcs.port]
        u8Value = srcs.IP
        fValue = srcs.Params
        all_values = u16Value + u8Value + fValue  # Concatenate lists first
        buf = struct.pack("4H4B6f", *all_values)   # Unpack the single combined list
        self.udp_socket.sendto(buf, (IP, 30100 + (copterID - 1) * 2))

    def sendImuReqCopterSim(self, copterID=1, IP="127.0.0.1", freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP of the PC to send request to
        port is the base port that CopterSim send to
        (Actual port for a vehicle = baseport + copterID -1)
        freq is the frequency of the send data
        This function will init a thread to listen IMU data
        """
        self.sendImuReqClient(copterID, IP, freq)
        self.sendImuReqServe(copterID)


    def sendImuReqClient(self, copterID=1, IP="127.0.0.1", freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP of the PC to send request to
        port is the base port that CopterSim send to
        (Actual port for a vehicle = baseport + copterID -1)
        freq is the frequency of the send data
        """

        # if RemotSendIP has been set, the IMU rev IP will be RemotSendIP
        # else use local IP address 127.0.0.1

        BackIP='127.0.0.1'
        if self.RemotSendIP != "":
            BackIP=self.RemotSendIP

        srcs = SensorReqCopterSim()
        srcs.sensorType = 0  # IMU sensor data
        srcs.updateFreq = freq

        # The return address defaults to 127.x.x.x, CopterSim will return to the original IP after receiving
        # If self.RemotSendIP is set, CopterSim will forward data to the specified port
        cList = BackIP.split(".")
        if len(cList) == 4:
            srcs.IP[0] = int(cList[0])
            srcs.IP[1] = int(cList[1])
            srcs.IP[2] = int(cList[2])
            srcs.IP[3] = int(cList[3])
        srcs.port = 31000 + copterID - 1
        self.sendReqToCopterSim(srcs, copterID,IP)  # Send message to request IMU data

    def sendImuReqServe(self, copterID=1):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        port is the base port that CopterSim send to
        (Actual port for a vehicle = baseport + copterID -1)
        This function will init a thread to listen IMU data
        """
        port = 31000 + copterID - 1
        self.udp_imu.bind(("0.0.0.0", port))
        self.tIMU = threading.Thread(target=self.getIMUDataLoop, args=(copterID,))
        self.tIMU.start()

    def getIMUDataLoop(self, copterID):
        global isEnableRosTrans
        global is_use_ros1

        for i in range(len(self.RflyTimeVect)):
            if self.RflyTimeVect[i].copterID == copterID:
                if isEnableRosTrans:
                    self.imu.rflyStartStmp = self.RflyTimeVect[i].rosStartTimeStmp
                else:
                    self.imu.rflyStartStmp = self.RflyTimeVect[i].pyStartTimeStmp

        # if self.tTimeStmpFlag

        print("Start listening to IMU Msg")
        while True:
            try:
                buf, addr = self.udp_imu.recvfrom(65500)
                if len(buf) == 40:
                    # print(len(buf[0:12]))
                    IMUData = struct.unpack("2i1d6f", buf)
                    if IMUData[0] == 1234567898:
                        self.imu.checksum = IMUData[0]
                        self.imu.seq = IMUData[1]
                        self.imu.timestmp = IMUData[2]

                        if self.imu.rflyStartStmp < 0.01:  # Indicates that CopterSim timestamp has not been obtained yet
                            print("No CopterSim time, use image time to calculate.")
                            if isEnableRosTrans:
                                if is_use_ros1:
                                    ros_now_time = rospy.Time.now().to_sec()
                                else:
                                    now = self.ros_node.get_clock().now()
                                    ros_now_time = (
                                    now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                    )
                                self.imu.rflyStartStmp = (
                                    ros_now_time - self.imu.timestmp - 0.005
                                )
                                # tStmp.rosStartTimeStmp = tStmp.pyStartTimeStmp + ros_now_time - time.time()
                            else:
                                self.imu.rflyStartStmp = (
                                    time.time() - self.imu.timestmp - 0.005
                                )

                        self.imu.imuStmp = self.imu.rflyStartStmp + self.imu.timestmp

                        if self.isPrintTime:
                            self.lastIMUTime = time.time()
                            print("IMU:", self.imu.timestmp)
                        self.imu.acc[0] = IMUData[3]
                        self.imu.acc[1] = IMUData[4]
                        self.imu.acc[2] = IMUData[5]
                        self.imu.rate[0] = IMUData[6]
                        self.imu.rate[1] = IMUData[7]
                        self.imu.rate[2] = IMUData[8]
                        if not self.hasIMUData:
                            self.hasIMUData = True
                            print("Got CopterSim IMU Msg!")
                        if isEnableRosTrans and self.hasIMUData:
                            if is_use_ros1:
                                self.imu.Imu2ros()
                            else:
                                self.imu.Imu2ros(self.ros_node)
                            # Push IMU message to ROS message

            except Exception as ex:
                print("Error to listen to IMU Msg!")
                print(ex)
                sys.exit(0)

    def StartTimeStmplisten(self):
        """Start to listen to 20005 port to get RflyTimeStmp of CopterID"""
        self.udp_time = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_time.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_time.bind(("0.0.0.0", 20005))
        # Add support for multicast port

        try:
            status = self.udp_time.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton("224.0.0.10") + socket.inet_aton("0.0.0.0"),
            )
        except:
            print('Failed to Init multicast!')
        self.tTimeStmp = threading.Thread(target=self.TimeStmploop, args=())
        self.tTimeStmpFlag = True
        self.tTimeStmp.start()

    def endTimeStmplisten(self):
        self.tTimeStmpFlag = False
        time.sleep(0.5)
        self.tTimeStmp.join()
        time.sleep(0.5)

    def TimeStmploop(self):
        global isEnableRosTrans
        global is_use_ros1
        print("Start listening to timeStmp Msg")
        self.udp_time.settimeout(3)
        while self.tTimeStmpFlag:
            try:
                buf, addr = self.udp_time.recvfrom(65500)
                if len(buf) == 32:
                    # print(len(buf[0:12]))
                    TimeData = struct.unpack("2i3q", buf)
                    if TimeData[0] == 123456789:
                        cpIDTmp = TimeData[1]
                        isTimeExist = False
                        for i in range(len(self.RflyTimeVect)):
                            if self.RflyTimeVect[i].copterID == cpIDTmp:
                                isTimeExist = True
                                # If it's already in the list, don't receive it again
                                # self.RflyTimeVect[i].Update(TimeData)
                                break

                        if not isTimeExist:
                            tStmp = RflyTimeStmp(TimeData)
                            CurPyTime = time.time()
                            print("Got time msg from CopterSim #", tStmp.copterID)
                            if (
                                CurPyTime - tStmp.SysCurrentTime > 0
                                and CurPyTime - tStmp.SysCurrentTime < 0.1
                            ):
                                tStmp.isCopterSimOnPC = True  # Indicates Python and CopterSim are on the same computer
                                tStmp.pyStartTimeStmp = tStmp.SysStartTime
                                print("CopterSim running on this PC")
                            else:  # Indicates this Python script and CopterSim are not on the same computer
                                tStmp.isCopterSimOnPC = False
                                tStmp.pyStartTimeStmp = (
                                    CurPyTime
                                    - tStmp.SysCurrentTime
                                    + tStmp.SysStartTime
                                    - 0.01
                                )
                                print("CopterSim not on this PC")
                                print(tStmp.SysCurrentTime)
                                print(CurPyTime)

                            if isEnableRosTrans:
                                if is_use_ros1:
                                    ros_now_time = rospy.Time.now().to_sec()
                                else:
                                    now = self.ros_node.get_clock().now()
                                    ros_now_time = (
                                        now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                    )
                                tStmp.rosStartTimeStmp = (
                                    tStmp.pyStartTimeStmp + ros_now_time - CurPyTime
                                )

                            self.RflyTimeVect = self.RflyTimeVect + [
                                copy.deepcopy(tStmp)
                            ]  # Extend list, add an element

            except:
                print("No Time Msg!")
                # Break the loop, stop listening
                break

    def sendUpdateUEImage(self, vs=VisionSensorReq(), windID=0, IP=""):
        if not isinstance(vs, VisionSensorReq):
            raise Exception("Wrong data input to addVisSensor()")
        
        # If it's a Linux system, get its own IP address
        if isLinux and (vs.SendProtocol[1]==127 or vs.SendProtocol[1]==0):
            ip=''
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                ip = s.getsockname()[0]
            except:
                ip=''
            finally:
                s.close()
            if ip!='':
                cList = ip.split(".")
                if len(cList) == 4:
                    vs.SendProtocol[1] = int(cList[0])
                    vs.SendProtocol[2] = int(cList[1])
                    vs.SendProtocol[3] = int(cList[2])
                    vs.SendProtocol[4] = int(cList[3])
         
        intValue = [
            vs.checksum,
            vs.SeqID,
            vs.TypeID,
            vs.TargetCopter,
            vs.TargetMountType,
            vs.DataWidth,
            vs.DataHeight,
            vs.DataCheckFreq,
        ] + vs.SendProtocol
        if self.isNewJson:  # Send using new protocol version
            floValue = (
                [vs.CameraFOV]
                + vs.SensorPosXYZ
                + [vs.EularOrQuat]
                + vs.SensorAngEular
                + vs.SensorAngQuat
                + vs.otherParams
            )
            combined = intValue+floValue
            buf = struct.pack("16H28f", *combined)
        else:  # Send using old protocol version
            floValue = (
                [vs.CameraFOV]
                + vs.SensorPosXYZ
                + vs.SensorAngEular
                + vs.otherParams[0:8]
            )
            buf = struct.pack("16H15f", *intValue, *floValue)
        if IP == "":  # If the IP of the CopterSim computer is specified, use this value
            IP = self.ip
        self.udp_socket.sendto(buf, (IP, 20010 + windID))
        if self.RemotSendIP != "" and self.RemotSendIP != "127.0.0.1":
            self.udp_socket.sendto(buf, (self.RemotSendIP, 20010 + windID))

    def sendUE4Cmd(self, cmd, windowID=-1):
        # If it's a str type, convert to bytes type
        if isinstance(cmd, str):
            cmd = cmd.encode()

        # print(type(cmd))
        if len(cmd) <= 51:
            buf = struct.pack("i52s", 1234567890, cmd)
        elif len(cmd) <= 249:
            buf = struct.pack("i252s", 1234567890, cmd)
        else:
            print("Error: Cmd is too long")
            return
        if windowID < 0:
            for i in range(3):  # Assume at most three windows are open
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can receive message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

    def sendReqToUE4(self, windID=0, IP=""):
        """send VisSensor list to RflySim3D to request image
        windID specify the index of RflySim3D window to send image
        """
        if IP == "":
            IP = self.ip

        if len(self.VisSensor) <= 0:
            print("Error: No sensor is obtained.")
            return False

        # EmptyMem = np.zeros(66,dtype=np.int).tolist()
        # buf = struct.pack("66i",*EmptyMem)
        # self.mm0.seek(0)
        # self.mm0.write(buf)
        # self.mm0.seek(0)
        contSeq0 = False
        if self.isUE4DirectUDP or self.RemotSendIP != "":
            for i in range(len(self.VisSensor)):
                if (
                    self.isUE4DirectUDP and self.VisSensor[i].SendProtocol[0] == 0
                ):  # If shared memory was previously set, force conversion to direct UDP sending
                    self.VisSensor[i].SendProtocol[0] = 1
                if self.RemotSendIP != "":
                    cList = self.RemotSendIP.split(".")
                    if len(cList) == 4:
                        self.VisSensor[i].SendProtocol[1] = int(cList[0])
                        self.VisSensor[i].SendProtocol[2] = int(cList[1])
                        self.VisSensor[i].SendProtocol[3] = int(cList[2])
                        self.VisSensor[i].SendProtocol[4] = int(cList[3])
                if self.VisSensor[i].SeqID == 0:
                    contSeq0 = True

        if contSeq0:
            self.sendUE4Cmd("RflyClearCapture", windID)

        for i in range(len(self.VisSensor)):
            # struct VisionSensorReq {
            # 	uint16 checksum; //Data checksum, 12345
            # 	uint16 SeqID; //Memory sequence ID
            # 	uint16 TypeID; //Sensor type ID
            # 	uint16 TargetCopter; //Bound target aircraft     //Changeable
            # 	uint16 TargetMountType; //Binding type    //Changeable
            # 	uint16 DataWidth;   //Data or image width
            # 	uint16 DataHeight; //Data or image height
            # 	uint16 DataCheckFreq; //Frequency for checking data updates
            # 	uint16 SendProtocol[8]; //Transmission type (shared memory, UDP uncompressed, UDP video stream), IP address, port number, ...
            # 	float CameraFOV;  //Camera field of view (visual sensors only)  //Changeable
            #   float EularOrQuat; //Select Euler angle or quaternion method, >0.5 means quaternion
            #   float SensorAngEular[3]; //Sensor installation angle   //Changeable
            #   float SensorAngQuat[4]; //Sensor installation quaternion   //Changeable
            #   float otherParams[16]; //Reserved 16 float data slots
            # }16H28f
            vs = self.VisSensor[i]
            
            # If it's a Linux system, get its own IP address
            if isLinux and (vs.SendProtocol[1]==127 or vs.SendProtocol[1]==0):
                ip=''
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 80))
                    ip = s.getsockname()[0]
                except:
                    ip=''
                finally:
                    s.close()
                if ip!='':
                    cList = ip.split(".")
                    if len(cList) == 4:
                        vs.SendProtocol[1] = int(cList[0])
                        vs.SendProtocol[2] = int(cList[1])
                        vs.SendProtocol[3] = int(cList[2])
                        vs.SendProtocol[4] = int(cList[3])
            
            intValue = [
                vs.checksum,
                vs.SeqID,
                vs.TypeID,
                vs.TargetCopter,
                vs.TargetMountType,
                vs.DataWidth,
                vs.DataHeight,
                vs.DataCheckFreq,
            ] + vs.SendProtocol
            if self.isNewJson:  # Send using new protocol version
                floValue = (
                    [vs.CameraFOV]
                    + vs.SensorPosXYZ
                    + [vs.EularOrQuat]
                    + vs.SensorAngEular
                    + vs.SensorAngQuat
                    + vs.otherParams
                )
                buf = struct.pack("16H28f", *intValue, *floValue)
            else:  # Send using old protocol version
                floValue = (
                    [vs.CameraFOV]
                    + vs.SensorPosXYZ
                    + vs.SensorAngEular
                    + vs.otherParams[0:8]
                )
                buf = struct.pack("16H15f", *intValue, *floValue)
            self.udp_socket.sendto(buf, (IP, 20010 + windID))

        time.sleep(1)

        if IP != "127.0.0.1" or isLinux:
            return True

        # struct UE4CommMemData {
        # 	int Checksum;//Checksum, set to 1234567890
        # 	int totalNum;//Maximum number of sensors
        # 	int WidthHeigh[64];//Resolution width/height sequence, contains up to 32 sensors
        # }
        if isLinux:  # Shared memory code for Linux
            # Linux mmap
            # SHARE_MEMORY_FILE_SIZE_BYTES = 66*4
            f = open("/dev/shm/UE4CommMemData", "r+b")
            fd = f.fileno()
            self.mm0 = mmap.mmap(fd, 66 * 4)
        else:  # Shared memory code for Windows
            self.mm0 = mmap.mmap(0, 66 * 4, "UE4CommMemData")  # Common area

        Data = np.frombuffer(self.mm0, dtype=np.int32)
        checksum = Data[0]
        totalNum = Data[1]
        ckCheck = False
        # print(Data)
        if checksum >= 1234567890 and checksum <= 1234568890:
            CamSeqIndex = checksum - 1234567890
            ckCheck = True
            for i in range(len(self.VisSensor)):
                isSucc = False
                vs = self.VisSensor[i]
                idx = vs.SeqID - CamSeqIndex * 32
                width = Data[2 + idx * 2]
                height = Data[2 + idx * 2 + 1]
                if width == vs.DataWidth and height == vs.DataHeight:
                    if idx <= totalNum:
                        isSucc = True
                if not isSucc:
                    ckCheck = False
                    break
        if not ckCheck:
            print("Error: Sensor req failed from UE4.")
            return False
        print("Sensor req success from UE4.")
        self.hasReqUE4 = True
        return True

    def img_udp_thrdNew(self, udpSok, idx, typeID):
        CheckSum = -1
        CheckSumSize = struct.calcsize("1i")
        fhead_len = 0
        imgPackUnit = 60000
        global isEnableRosTrans
        global is_use_ros1
        Frameid = -1
        seqList = []
        dataList = []
        timeList = []
        recPackNum = 0
        timeStmpStore = 0
        IsReframe = False  # To determine if a frame has been received repeatedly
        no_fid_len = struct.calcsize("4i1d")
        fid_len = struct.calcsize("6i1d")
        dd = None
        while True:
            if isEnableRosTrans and ((is_use_ros1 and rospy.is_shutdown())):
                break
            try:
                buf, addr = udpSok.recvfrom(imgPackUnit + 2000)  # Add some margin to ensure header data is included
            except socket.error:
                continue

            if CheckSum == -1:  # Initialize CheckSum for the first frame
                CheckSum = struct.unpack("1i", buf[0:CheckSumSize])
                if CheckSum[0] == 1234567890:  # Data without frame ID
                    fhead_len = no_fid_len
                    Frameid == 0
                if CheckSum[0] == 1234567893:  # Data with frame ID
                    fhead_len = fid_len
                    Frameid == 0
            if len(buf) < fhead_len:  # If packet length is less than header length, data error
                print("img_udp_thrdNew len(buf)<fhead_size")
                continue
            if fhead_len == fid_len:
                dd = struct.unpack("6i1d", buf[0:fhead_len])  # Checksum, packet length, packet sequence, total packets, timestamp
                if dd[-3] != Frameid and dd[2] == 0:  # Update frame ID
                    if dd[-3] < 0:
                        print(
                            "\033[31m frame id less than zero ! \033[0m"
                        )  # Sender didn't handle int overflow
                    Frameid = dd[-3]
                    IsReFrame = False
                elif dd[-3] == Frameid and dd[2] == 0:
                    # print("have same frame received")  # Same frame received multiple times
                    IsReFrame = True
                    continue
                if IsReFrame:  # If a frame is split into multiple packets, non-zero packets of a repeated frame also don't need processing
                    continue
            if fhead_len == no_fid_len:
                dd = struct.unpack("4i1d", buf[0:fhead_len])  # Checksum, packet length, packet sequence, total packets, timestamp
            if dd == None:
                print("\033[31m Protocol error\033[0m")  # Communication protocol mismatch
                continue
            if dd[0] != CheckSum[0] or dd[1] != len(buf):  # Incorrect checksum or length
                print("\033[31m Wrong Data!\033[0m")
                continue
            packSeq = dd[2]  # Packet sequence number
            if packSeq == 0:  # If it's the first packet
                seqList = []  # Clear data sequence list
                dataList = []  # Clear data buffer list
                seqList = seqList + [packSeq]  # Extract sequence number
                dataList = dataList + [buf[fhead_len:]]  # Extract remaining data after header
                timeStmpStore = dd[-1]  # The last one is the timestamp
                recPackNum = dd[3]  # Use total packets defined in header as reception end flag
            else:  # If not the header, directly store it in the list
                if recPackNum == 0:
                    continue
                # If timestamps don't match
                if not math.isclose(timeStmpStore, dd[-1], rel_tol=0.00001):
                    continue  # Skip this packet
                seqList = seqList + [packSeq]  # Extract sequence number
                dataList = dataList + [buf[fhead_len:]]  # Extract remaining data after header
            # if typeID==2:
            # print(seqList,recPackNum,len(dataList))
            if len(seqList) == recPackNum:  # If the number of received packets reaches the total, start processing the image
                recPackNum = 0
                # print('Start Img Cap')
                data_total = b""
                dataOk = True
                for i in range(len(seqList)):
                    if seqList.count(i) < 1:
                        dataOk = False  # If a sequence number is not in the packet, report error
                        print("\033[31m Failed to process img pack \033[0m")
                        break
                    idx0 = seqList.index(i)  # Search for packet sequence number in order
                    data_total = data_total + dataList[idx0]
                #if typeID==2:
                #    print(len(data_total))
                if dataOk:  # If all data is fine, start processing the image
                    #if typeID==23:
                    #   print('Start img cap',self.VisSensor[idx].SendProtocol[0])
                    if (
                        self.VisSensor[idx].SendProtocol[0] == 1
                        or self.VisSensor[idx].SendProtocol[0] == 3
                    ):
                        if (
                            self.VisSensor[idx].TypeID == 1
                            or self.VisSensor[idx].TypeID == 2
                            or self.VisSensor[idx].TypeID == 3
                            or self.VisSensor[idx].TypeID == 4
                            or self.VisSensor[idx].TypeID == 40
                            or self.VisSensor[idx].TypeID == 41
                        ):
                            nparr = np.frombuffer(data_total, np.uint8)
                            colorType = cv2.IMREAD_COLOR
                            if typeID == 2:
                                colorType = cv2.IMREAD_ANYDEPTH
                            elif typeID == 3 or typeID == 40:
                                colorType = cv2.IMREAD_GRAYSCALE
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = cv2.imdecode(nparr, colorType)
                            self.Img_lock[idx].release()
                            if self.Img[idx] is None:
                                print("\033[31m Wrong Img decode! \033[0m")
                                self.hasData[idx] = False
                            else:
                                self.hasData[idx] = True
                                self.timeStmp[idx] = timeStmpStore

                                if (
                                    self.rflyStartStmp[idx] < 0.01
                                ):  # Indicates that CopterSim timestamp has not been obtained yet
                                    print(
                                        "No CopterSim time, use image time to calculate."
                                    )
                                    if isEnableRosTrans:
                                        if is_use_ros1:
                                            ros_now_time = rospy.Time.now().to_sec()
                                        else:
                                            now = self.ros_node.get_clock().now()
                                            ros_now_time = (
                                                now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                            )
                                        self.rflyStartStmp[idx] = (
                                            ros_now_time - self.timeStmp[idx] - 0.01
                                        )
                                        # tStmp.rosStartTimeStmp = tStmp.pyStartTimeStmp + ros_now_time - time.time()
                                    else:
                                        self.rflyStartStmp[idx] = (
                                            time.time() - self.timeStmp[idx] - 0.01
                                        )

                                self.imgStmp[idx] = (
                                    self.rflyStartStmp[idx] + self.timeStmp[idx]
                                )

                                if self.isPrintTime:
                                    dTime = time.time() - self.lastIMUTime
                                    print(
                                        "Img",
                                        idx,
                                        ":",
                                        "{:.5f}".format(timeStmpStore),
                                        ", dTimeIMU: ",
                                        dTime,
                                    )
                                    print("frame_id: ", Frameid, "idx: ", idx)

                        if self.VisSensor[idx].SendProtocol[0] == 1 and (
                            self.VisSensor[idx].TypeID == 20
                            or self.VisSensor[idx].TypeID == 21
                            or self.VisSensor[idx].TypeID == 22
                            or self.VisSensor[idx].TypeID == 23
                        ):
                            # print('')
                            posAng = np.frombuffer(
                                data_total, dtype=np.float32, count=6
                            )  # pos ang  6*4
                            PointNum = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset=4 * 6
                            )  # num 4*1
                            PointNum = PointNum[0]
                            # print('PointNum: ',PointNum)
                            # print('posAng: ', posAng)
                            self.ImgData[idx] = posAng.tolist() + [PointNum]

                            L = np.frombuffer(
                                data_total,
                                dtype=np.int16,
                                count=PointNum
                                * 4,  # --Lidar PointNum * 3, Fix PointNum * 4
                                offset=4 * 7,
                            )  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = L.reshape(
                                PointNum, 4
                            )  # --Lidar L.reshape(PointNum, 3)
                            self.Img[idx] = (
                                self.Img[idx]
                                / 32767.0
                                * self.VisSensor[idx].otherParams[0]
                            )

                            self.Img_lock[idx].release()
                            self.hasData[idx] = True
                            self.timeStmp[idx] = timeStmpStore
                            if (self.rflyStartStmp[idx] < 0.01):
                                # Indicates that CopterSim timestamp has not been obtained yet
                                print(
                                    "No CopterSim time, use image time to calculate."
                                )
                                if isEnableRosTrans:
                                    if is_use_ros1:
                                        ros_now_time = rospy.Time.now().to_sec()
                                    else:
                                        now = self.ros_node.get_clock().now()
                                        ros_now_time = (
                                            now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                        )
                                    self.rflyStartStmp[idx] = (
                                        ros_now_time - self.timeStmp[idx] - 0.01
                                    )
                                    # tStmp.rosStartTimeStmp = tStmp.pyStartTimeStmp + ros_now_time - time.time()
                                else:
                                    self.rflyStartStmp[idx] = (
                                        time.time() - self.timeStmp[idx] - 0.01
                                    )

                            self.imgStmp[idx] = (
                                self.rflyStartStmp[idx] + self.timeStmp[idx])

                            if self.isPrintTime:
                                dTime = time.time() - self.lastIMUTime
                                print(
                                        "Img",
                                        idx,
                                        ":",
                                        "{:.5f}".format(timeStmpStore),
                                        ", dTimeIMU: ",
                                        dTime,
                                    )
                                print("frame_id: ", Frameid, "idx: ", idx)
                        if self.VisSensor[idx].TypeID == 5:
                            if not isinstance(self.DistanceSensor, DistanceSensor):
                                self.DistanceSensor = DistanceSensor()
                            self.DistanceSensor.Distance = np.frombuffer(
                                data_total, dtype=np.float32, count=1, offset=0
                            )
                            self.DistanceSensor.CopterID = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset=4
                            )
                            self.DistanceSensor.RayStart = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset=8
                            )
                            self.DistanceSensor.Ang = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset=20
                            )
                            self.DistanceSensor.ImpactPoint = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset=32
                            )
                            self.DistanceSensor.BoxOri = np.frombuffer(
                                data_total, dtype=np.float32, count=3, offset=44
                            )

                    elif self.VisSensor[idx].SendProtocol[0] == 2:
                        dtyp = np.uint8
                        dim = 3
                        if typeID == 1 or typeID == 4 or typeID == 41:
                            dtyp = np.uint8
                            dim = 3
                        elif typeID == 2:
                            dtyp = np.uint16
                            dim = 1
                        elif typeID == 3 or typeID == 40:
                            dtyp = np.uint8
                            dim = 1
                        DataWidth = self.VisSensor[idx].DataWidth
                        DataHeight = self.VisSensor[idx].DataHeight
                        L = np.frombuffer(data_total, dtype=dtyp)
                        # colorType=cv2.IMREAD_COLOR
                        # if typeID==2 or typeID==3:
                        #     colorType=cv2.IMREAD_GRAYSCALE
                        # self.Img[idx] = cv2.imdecode(nparr, colorType)
                        self.Img_lock[idx].acquire()
                        self.Img[idx] = L.reshape(DataHeight, DataWidth, dim)
                        self.Img_lock[idx].release()
                        self.hasData[idx] = True
                        self.timeStmp[idx] = timeStmpStore
                        if self.isPrintTime:
                            dTime = time.time() - self.lastIMUTime
                            print("Img", idx, ":", timeStmpStore, ", dTimeIMU: ", dTime)

                    if isEnableRosTrans and self.hasData[idx]:  # If ROS message needs to be published
                        if self.VisSensor[idx].TypeID >= 1:  # Currently all image capture operations are performed simultaneously
                            self.imu.AlignTime(timeStmpStore)  # Send timestamp to IMU publishing thread
                        seq_id = str(self.VisSensor[idx].SeqID)

                        if self.time_record[idx] < 0.0000001:
                            self.time_record[idx] = timeStmpStore
                            if is_use_ros1:
                                self.rostime[idx] = rospy.Time.now()
                            else:
                                self.rostime[idx] = self.ros_node.get_clock().now()
                            continue

                        type_id = self.VisSensor[idx].TypeID
                        if (
                            type_id == 1
                            or type_id == 2
                            or type_id == 3
                            or type_id == 4
                            or type_id == 40
                            or type_id == 41
                        ):
                            encoding_ = "bgr8"
                            # type = sensor.Image
                            if not seq_id in self.sensor_data.keys():
                                self.sensor_data[seq_id] = sensor.Image()
                                frame_id = "map"
                                if len(self.VisSensor) == self.sensors_num:
                                    frame_id = self.sensors_frame_id[idx]
                                self.sensor_data[seq_id].header.frame_id = frame_id
                            if is_use_ros1:
                                self.sensor_data[seq_id].header.stamp = rospy.Duration(
                                    self.imgStmp[idx]
                                )
                            else:
                                rclpy_time = self.rostime[idx] + Duration(
                                    seconds=self.imgStmp[idx],
                                    nanoseconds=0,
                                )
                                #self.sensor_data[
                                #    seq_id
                                #].header.stamp = rclpy_time.to_msg()
                                seconds, nanoseconds = rclpy_time.seconds_nanoseconds()
                                self.sensor_data[
                                   seq_id
                                ].header.stamp.sec =  int(seconds & 0x7FFFFFFF)
                                self.sensor_data[
                                   seq_id
                                ].header.stamp.nanosec =  nanoseconds
                                self.sensor_data[seq_id].header.frame_id = frame_id
                            # self.sensor_data[seq_id].header.seq = Frameid #ROS2 haven't this val
                            byte_num = 1
                            if type_id == 1:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_rgb"
                                    )
                                # msg.encoding = "bgr8"
                                byte_num = 3
                            elif type_id == 2:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_depth"
                                    )
                                encoding_ = "mono16"
                                byte_num = 2
                            elif type_id == 3:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_gray"
                                    )
                                encoding_ = "mono8"
                            elif type_id == 40:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor"
                                        + seq_id
                                        + "/img_Infrared_Gray"
                                    )
                                encoding_ = "mono8"
                            elif type_id == 4:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_Segmentation"
                                    )
                                byte_num = 3
                            elif type_id == 41:
                                if not seq_id in self.topic_name.keys():
                                    self.topic_name[seq_id] = (
                                        "/rflysim/sensor" + seq_id + "/img_Infrared"
                                    )
                                byte_num = 3

                            if is_use_ros1:
                                # In fact, ROS1 images can also be converted using cv_bridge, but ROS1 versions before Ubuntu 18.04 do not support Python 3 by default,
                                # Forcing the use of Python 3's cv_bridge interface for conversion can cause compatibility issues when receiving topics in ROS C++. Considering this, the most basic assignment method is used here.
                                self.sensor_data[seq_id].height = self.Img[idx].shape[0]
                                self.sensor_data[seq_id].width = self.Img[idx].shape[1]
                                self.sensor_data[seq_id].encoding = encoding_
                                self.sensor_data[seq_id].data = self.Img[idx].tostring()
                                self.sensor_data[seq_id].step = (
                                    self.sensor_data[seq_id].width * byte_num
                                )
                            else:
                                # Using cv2_to_imgmsg here actually calls the C++ interface, which is faster in processing than the above method and saves more CPU resources
                                # cv2.imshow("rosdata",self.Img[idx])
                                # cv2.waitKey(1)
                                self.sensor_data[seq_id] = self.cv_bridge.cv2_to_imgmsg(self.Img[idx], encoding= encoding_)
                                self.sensor_data[seq_id].header.frame_id = frame_id

                        if type_id == 20 or type_id == 21 or type_id == 22 or type_id == 23:
                            if type_id == 20 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/vehicle_lidar"
                                )
                            if type_id == 21 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/global_lidar"
                                )
                            if type_id == 22 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/livox_lidar"
                                )
                            if type_id == 23 and not seq_id in self.topic_name.keys():
                                self.topic_name[seq_id] = (
                                    "/rflysim/sensor" + seq_id + "/mid360_lidar"
                                )
                            if not seq_id in self.sensor_data.keys():
                                # type = sensor.PointCloud2
                                msg = sensor.PointCloud2()
                                msg.height = 1
                                msg.width = self.Img[idx].shape[0]
                                msg.fields = [
                                    sensor.PointField(
                                        "x", 0, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "y", 4, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "z", 8, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "w", 12, sensor.PointField.FLOAT32, 1
                                    ),
                                ]
                                msg.is_bigendian = False
                                msg.point_step = 16
                                msg.is_dense = False
                                self.sensor_data[seq_id] = msg
                                frame_id = "map"
                                if len(self.VisSensor) == self.sensors_num:
                                    frame_id = self.sensors_frame_id[idx]
                                self.sensor_data[seq_id].header.frame_id = frame_id
                            # self.sensor_data[seq_id].header.seq = Frameid
                            self.sensor_data[seq_id].width = self.Img[idx].shape[0]
                            self.sensor_data[seq_id].row_step = (
                                self.sensor_data[seq_id].point_step
                                * self.Img[idx].shape[0]
                            )

                            self.sensor_data[seq_id].data = np.asarray(
                                self.Img[idx], np.float32
                            ).tostring()

                        if is_use_ros1:
                            if not self.topic_name[seq_id] in self.sensor_pub.keys():
                                self.sensor_pub[
                                    self.topic_name[seq_id]
                                ] = rospy.Publisher(
                                    self.topic_name[seq_id],
                                    type(self.sensor_data[seq_id]),
                                    queue_size=10,
                                )
                            self.sensor_data[seq_id].header.stamp = rospy.Duration(
                                self.imgStmp[idx]
                            )
                            self.sensor_pub[self.topic_name[seq_id]].publish(
                                self.sensor_data[seq_id]
                            )
                        else:
                            if not self.topic_name[seq_id] in self.sensor_pub.keys():
                                self.sensor_pub[
                                    self.topic_name[seq_id]
                                ] = self.ros_node.create_publisher(
                                    type(self.sensor_data[seq_id]),
                                    self.topic_name[seq_id],
                                    qos_profile=qos_profile_sensor_data,
                                )
                            rclpy_time = self.rostime[idx] + Duration(
                                seconds=(timeStmpStore - self.time_record[idx]),
                                nanoseconds=0,
                            )
                            # self.sensor_data[seq_id].header.stamp = rclpy_time.to_msg()
                            seconds, nanoseconds = rclpy_time.seconds_nanoseconds()
                            self.sensor_data[seq_id].header.stamp.sec = int(seconds & 0x7FFFFFFF)
                            self.sensor_data[seq_id].header.stamp.nanosec = nanoseconds
                            self.sensor_pub[self.topic_name[seq_id]].publish(
                                self.sensor_data[seq_id]
                            )
        udpSok.close()

    def img_mem_thrd(self, idxList):
        global isEnableRosTrans
        global is_use_ros1
        mmList = []
        for i in range(len(idxList)):
            idx = idxList[i]
            SeqID = self.VisSensor[idx].SeqID
            DataWidth = self.VisSensor[idx].DataWidth
            DataHeight = self.VisSensor[idx].DataHeight
            typeID = self.VisSensor[idx].TypeID
            dim = 3
            dimSize = 1
            otherSize = 0
            if typeID == 1 or typeID == 4 or typeID == 41:
                dim = 3
                dimSize = 1
            elif typeID == 2:
                dim = 1
                dimSize = 2
            elif typeID == 3 or typeID == 40:
                dim = 1
                dimSize = 1
            elif typeID == 20 or typeID == 21 or typeID == 22:
                # dim = 3
                dim = 4  # --Lidar  dim = 3
                dimSize = 2
                otherSize = 4 * 7
            elif typeID == 5:  # 1 + 8 + 14 * 4
                DataWidth = 0
                DataHeight = 0
                dim = 0
                dimSize = 0
                otherSize = 14 * 4
            elif typeID == 23:
                DataWidth = 64
                DataHeight = 272
                dim = 4  # --Lidar  dim = 3
                dimSize = 2
                otherSize = 4 * 7

            if isLinux:
                # Linux
                dataLen = DataWidth * DataHeight * dim * dimSize + 1 + 8 + otherSize
                f = open("/dev/shm/" + "RflySim3DImg_" + str(SeqID), "r+b")
                fd = f.fileno()
                mm = mmap.mmap(fd, dataLen)
                # mm = mmap_file.read(SHARE_MEMORY_FILE_SIZE_BYTES)
            else:
                mm = mmap.mmap(
                    0,
                    DataWidth * DataHeight * dim * dimSize + 1 + 8 + otherSize,
                    "RflySim3DImg_" + str(SeqID),
                )
            mmList = mmList + [mm]
        # cv2.IMWRITE_PAM_FORMAT_GRAYSCALE
        while True:
            if isEnableRosTrans and ((is_use_ros1 and rospy.is_shutdown())):
                break
            for kk in range(len(idxList)):
                mm = mmList[kk]
                idx = idxList[kk]
                DataWidth = self.VisSensor[idx].DataWidth
                DataHeight = self.VisSensor[idx].DataHeight
                typeID = self.VisSensor[idx].TypeID
                dtyp = np.uint8
                dim = 3
                if typeID == 1 or typeID == 4 or typeID == 41:
                    dtyp = np.uint8
                    dim = 3
                elif typeID == 2:
                    dtyp = np.uint16
                    dim = 1
                elif typeID == 3 or typeID == 40:
                    dtyp = np.uint8
                    dim = 1
                elif typeID == 20 or typeID == 21 or typeID == 22 or typeID == 23:
                    dtyp = np.int16
                    dim = 4  # --Lidar   dim = 3
                elif typeID == 5:
                    dtyp = np.float32
                    dim = 0
                for ii in range(3):  # Try to read the memory area three times
                    flag = np.frombuffer(mm, dtype=np.uint8, count=1)
                    # print(flag[0])
                    if flag[0] == 2:  # Image writing completed
                        # print(flag[0])
                        mm.seek(0)
                        mm.write_byte(3)  # Enter reading state

                        # Start reading image
                        # L=np.frombuffer(mm,dtype = np.uint8)
                        # struct.unpack('d',L[1:9]) #Get timestamp
                        self.timeStmp[idx] = np.frombuffer(
                            mm, dtype=np.float64, count=1, offset=1
                        )

                        if self.rflyStartStmp[idx] < 0.01:  # Indicates that CopterSim timestamp has not been obtained yet
                            print("No CopterSim time, use image time to calculate.")
                            if isEnableRosTrans:
                                if is_use_ros1:
                                    ros_now_time = rospy.Time.now().to_sec()
                                else:
                                    now = self.ros_node.get_clock().now()
                                    ros_now_time = (
                                        now.to_msg().sec + now.to_msg().nanosec * 1e-9
                                    )
                                self.rflyStartStmp[idx] = (
                                    ros_now_time - self.timeStmp[idx] - 0.01
                                )
                                # tStmp.rosStartTimeStmp = tStmp.pyStartTimeStmp + ros_now_time - time.time()
                            else:
                                self.rflyStartStmp[idx] = (
                                    time.time() - self.timeStmp[idx] - 0.01
                                )

                        self.imgStmp[idx] = self.rflyStartStmp[idx] + self.timeStmp[idx]

                        if self.isPrintTime:
                            dTime = time.time() - self.lastIMUTime
                            print(
                                "Img",
                                idx,
                                ":",
                                self.timeStmp[idx],
                                ", dTimeIMU: ",
                                dTime,
                            )

                        mm.seek(0)
                        mm.write_byte(4)  # Enter read completion state
                        if (
                            typeID == 1
                            or typeID == 2
                            or typeID == 3
                            or typeID == 4
                            or typeID == 40
                            or typeID == 41
                        ):
                            L = np.frombuffer(mm, dtype=dtyp, offset=9)
                            # reshape array to 4 channel image array H X W X 4
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = L.reshape(DataHeight, DataWidth, dim)
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                self.sendImgUDPNew(idx)

                        elif typeID == 20 or typeID == 21 or typeID == 22 or typeID == 23:
                            posAng = np.frombuffer(
                                mm, dtype=np.float32, count=6, offset=9
                            )  # pos ang
                            PointNum = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9 + 4 * 6
                            )  # num
                            PointNum = PointNum[0]
                            # print('PointNum: ',PointNum)
                            # print('posAng: ', posAng)
                            self.ImgData[idx] = posAng.tolist() + [PointNum]
                            L = np.frombuffer(
                                mm, dtype=dtyp, count=PointNum * dim, offset=9 + 4 * 7
                            )  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img_lock[idx].acquire()
                            self.Img[idx] = L.reshape(PointNum, dim)
                            self.Img[idx] = (
                                self.Img[idx]
                                / 32767.0
                                * self.VisSensor[idx].otherParams[0]
                            )
                            self.Img_lock[idx].release()
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                # pos ang num cloud
                                L = np.frombuffer(
                                    mm,
                                    dtype=np.uint8,
                                    count=PointNum * dim * 2 + 4 * 7,
                                    offset=9,
                                )
                                self.sendImgBuffer(idx, L.tostring())
                        elif typeID == 5:
                            if not isinstance(self.DistanceSensor, DistanceSensor):
                                self.DistanceSensor = DistanceSensor()
                            flag = np.frombuffer(mm, dtype=np.uint8, count=1, offset=0)
                            curtime = np.frombuffer(
                                mm, dtype=np.float64, count=1, offset=1
                            )
                            self.DistanceSensor.Distance = np.frombuffer(
                                mm, dtype=np.float32, count=1, offset=9
                            )
                            self.DistanceSensor.CopterID = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=13
                            )
                            self.DistanceSensor.RayStart = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=17
                            )
                            self.DistanceSensor.Ang = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=29
                            )
                            self.DistanceSensor.ImpactPoint = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=41
                            )
                            self.DistanceSensor.BoxOri = np.frombuffer(
                                mm, dtype=np.float32, count=3, offset=53
                            )
                        # Exit the for loop after reading the image
                        # print("readImg"+str(idx))
                        # Linux ROS topic publishing
                        if isEnableRosTrans and self.hasData[idx]:  # If ROS message needs to be published
                            if self.VisSensor[idx].TypeID >= 1:  # Currently all image capture operations are performed simultaneously
                                self.imu.AlignTime(self.timeStmp[idx])  # Send timestamp to IMU publishing thread
                            topic_name = "/rflysim/sensor" + str(
                                self.VisSensor[idx].SeqID
                            )
                            frame_id = "map"  # For convenient visualization, use the default frame_id; modify according to actual conditions when used in algorithms
                            # For convenient visualization, use the default frame_id; modify according to actual conditions when used in algorithms
                            if len(self.VisSensor) == self.sensors_num:
                                frame_id = self.sensors_frame_id[idx]
                            header = std_msg.Header()
                            # header.seq = Frameid
                            header.frame_id = frame_id
                            if is_use_ros1:
                                header.stamp = rospy.Duration(self.imgStmp[idx])
                            else:
                                rclpy_time = Duration(
                                    seconds=(self.imgStmp[idx]),
                                    nanoseconds=0,
                                )
                                seconds, nanoseconds = rclpy_time.seconds_nanoseconds()
                                header.stamp.sec = int(seconds &0x7FFFFFFF)
                                header.stamp.nanosec  = nanoseconds

                            type_id = self.VisSensor[idx].TypeID

                            # print('Img',idx,':',header.stamp.to_sec())

                            type = Any
                            msg = Any
                            if (
                                type_id == 1
                                or type_id == 2
                                or type_id == 3
                                or type_id == 4
                                or type_id == 40
                                or type_id == 41
                            ):
                                encoding_ = "bgr8"
                                type = sensor.Image
                                msg = sensor.Image()
                                byte_num = 1
                                msg.header = header
                                if type_id == 1:
                                    topic_name += "/img_rgb"
                                    # msg.encoding = "bgr8"
                                    byte_num = 3
                                elif type_id == 2:
                                    encoding_ = "mono16"
                                    topic_name += "/img_depth"
                                    byte_num = 2
                                elif type_id == 3:
                                    encoding_ = "mono8"
                                    topic_name += "/img_gray"
                                elif type_id == 40:
                                    encoding_ = "mono8"
                                    topic_name += "/img_Infrared_Gray"
                                elif type_id == 4:
                                    topic_name += "/img_Segmentation"
                                    byte_num = 3
                                else:
                                    topic_name += "/img_Infrared"
                                    byte_num = 3
                                msg.height = self.Img[idx].shape[0]
                                msg.width = self.Img[idx].shape[1]
                                msg.encoding = encoding_
                                msg.data = self.Img[idx].tostring()
                                msg.step = msg.width * byte_num
                                # print(encoding_)
                            if type_id == 20 or type_id == 21 or type_id == 22 or type_id == 23:
                                type = sensor.PointCloud2
                                msg = sensor.PointCloud2()
                                msg.header = header
                                if type_id == 20:
                                    topic_name += "/vehicle_lidar"  # Topic /rflysim/senso3/vehicle_lidar
                                if type_id == 21:
                                    topic_name += "/global_lidar"
                                if type_id == 22:
                                    topic_name += "/livox_lidar"
                                if type_id == 23:
                                    topic_name += "/mid360_lidar"
                                msg.height = 1
                                msg.width = self.Img[idx].shape[0]
                                msg.fields = [
                                    sensor.PointField(
                                        "x", 0, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "y", 4, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(
                                        "z", 8, sensor.PointField.FLOAT32, 1
                                    ),
                                    sensor.PointField(  # --Lidar
                                        "w", 12, sensor.PointField.FLOAT32, 1
                                    ),
                                ]
                                msg.is_bigendian = False
                                msg.point_step = 16  # --Lidar
                                msg.row_step = msg.point_step * self.Img[idx].shape[0]
                                msg.is_dense = False
                                msg.data = np.asarray(
                                    self.Img[idx], np.float32
                                ).tostring()
                            if is_use_ros1:
                                if not topic_name in self.sensor_pub.keys():
                                    self.sensor_pub[topic_name] = rospy.Publisher(
                                        topic_name, type, queue_size=10
                                    )
                                self.sensor_pub[topic_name].publish(msg)
                            else:
                                if not topic_name in self.sensor_pub.keys():
                                    self.sensor_pub[
                                        topic_name
                                    ] = self.ros_node.create_publisher(
                                        type, topic_name, 10
                                    )
                                self.sensor_pub[topic_name].publish(msg)

                        break
            time.sleep(0.001)

    def startImgCap(self, isRemoteSend=False):
        """start loop to receive image from UE4,
        isRemoteSend=true will forward image from memory to UDP port
        """
        self.isRemoteSend = isRemoteSend
        global isEnableRosTrans
        memList = []
        udpList = []
        if isEnableRosTrans:
            self.time_record = np.zeros(len(self.VisSensor))
            if is_use_ros1:
                self.rostime = np.ndarray(len(self.time_record), dtype=rospy.Time)
            else:
                self.rostime = np.ndarray(len(self.time_record), dtype=rclpy.time.Time)

        for i in range(len(self.VisSensor)):
            self.Img = self.Img + [0]
            self.Img_lock = self.Img_lock + [
                threading.Lock()
            ]  # Each sensor is an independent thread, so an independent lock should be used
            self.ImgData = self.ImgData + [0]
            self.hasData = self.hasData + [False]
            self.timeStmp = self.timeStmp + [0]
            self.imgStmp = self.imgStmp + [0]

            TarCopt = self.VisSensor[i].TargetCopter
            starTime = 0
            for j in range(len(self.RflyTimeVect)):
                if self.RflyTimeVect[j].copterID == TarCopt:
                    if isEnableRosTrans:
                        starTime = self.RflyTimeVect[j].rosStartTimeStmp
                    else:
                        starTime = self.RflyTimeVect[j].pyStartTimeStmp
                    print("Got start time for SeqID #", self.VisSensor[i].SeqID)
            self.rflyStartStmp = self.rflyStartStmp + [starTime]
            IP = (
                str(self.VisSensor[i].SendProtocol[1])
                + "."
                + str(self.VisSensor[i].SendProtocol[2])
                + "."
                + str(self.VisSensor[i].SendProtocol[3])
                + "."
                + str(self.VisSensor[i].SendProtocol[4])
            )
            if IP == "0.0.0.0":
                IP = "127.0.0.1"
            if self.RemotSendIP != "":
                IP = self.RemotSendIP
            self.IpList = self.IpList + [IP]
            self.portList = self.portList + [self.VisSensor[i].SendProtocol[5]]
            if self.VisSensor[i].SendProtocol[0] == 0:
                memList = memList + [i]
            else:
                udpList = udpList + [i]

        if len(memList) > 0:
            self.t_menRec = threading.Thread(target=self.img_mem_thrd, args=(memList,))
            self.t_menRec.start()

        if len(udpList) > 0:
            # print('Enter UDP capture')
            for i in range(len(udpList)):
                udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 60000 * 100)
                udp.bind(("0.0.0.0", self.portList[udpList[i]]))
                typeID = self.VisSensor[udpList[i]].TypeID
                t_udpRec = threading.Thread(
                    target=self.img_udp_thrdNew,
                    args=(
                        udp,
                        udpList[i],
                        typeID,
                    ),
                )
                t_udpRec.start()

    def sendImgUDPNew(self, idx):
        img_encode = cv2.imencode(".png", self.Img[idx])[1]
        data_encode = np.array(img_encode)
        data = data_encode.tostring()
        self.sendImgBuffer(idx, data)

    def sendImgBuffer(self, idx, data):
        imgPackUnit = 60000
        imgLen = len(data)
        imgpackNum = imgLen // imgPackUnit + 1
        IP = self.IpList[idx]
        if self.RemotSendIP != "":
            IP = self.RemotSendIP

        CheckSum = 1234567890
        timeStmpSend = self.timeStmp[idx]

        # Loop to send image stream
        for i in range(imgpackNum):
            dataSend = []
            if imgPackUnit * (i + 1) > len(data):  # Extract data for the last packet
                dataSend = data[imgPackUnit * i :]
            else:  # Directly extract 60000 data for previous packets
                dataSend = data[imgPackUnit * i : imgPackUnit * (i + 1)]
            PackLen = 4 * 4 + 8 * 1 + len(dataSend)  # Length of fhead (4i1d) plus image data length
            fhead = struct.pack(
                "4i1d", CheckSum, PackLen, i, imgpackNum, timeStmpSend
            )  # Checksum, packet length, packet sequence, total packets, timestamp
            dataSend = fhead + dataSend  # Header plus image data
            self.udp_socket.sendto(dataSend, (IP, self.portList[idx]))  # Send out

    def jsonLoad(self, ChangeMode=-1, jsonPath=""):
        """load config.json file to create camera list for image capture,
        if ChangeMode>=0, then the SendProtocol[0] will be set to ChangeMode to change the transfer style
        """
        # print(sys.path[0])
        if os.path.isabs(jsonPath):
            print("Json use absolute path mode")
        else:
            print("Json use relative path mode")
            if len(jsonPath) == 0:
                jsonPath = sys.path[0] + "/Config.json"
            else:
                jsonPath = sys.path[0] + "/" + jsonPath

        print("jsonPath=", jsonPath)

        if not os.path.exists(jsonPath):
            print("The json file does not exist!")
            return False
        self.isNewJson = False
        with open(jsonPath, "r", encoding="utf-8") as f:
            jsData = json.loads(f.read())
            if len(jsData["VisionSensors"]) <= 0:
                print("No sensor data is found!")
                return False
            for i in range(len(jsData["VisionSensors"])):
                visSenStruct = VisionSensorReq()
                if isinstance(jsData["VisionSensors"][i]["SeqID"], int):
                    visSenStruct.SeqID = jsData["VisionSensors"][i]["SeqID"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TypeID"], int):
                    visSenStruct.TypeID = jsData["VisionSensors"][i]["TypeID"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TargetCopter"], int):
                    visSenStruct.TargetCopter = jsData["VisionSensors"][i][
                        "TargetCopter"
                    ]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TargetMountType"], int):
                    visSenStruct.TargetMountType = jsData["VisionSensors"][i][
                        "TargetMountType"
                    ]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataWidth"], int):
                    visSenStruct.DataWidth = jsData["VisionSensors"][i]["DataWidth"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataHeight"], int):
                    visSenStruct.DataHeight = jsData["VisionSensors"][i]["DataHeight"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataCheckFreq"], int):
                    visSenStruct.DataCheckFreq = jsData["VisionSensors"][i][
                        "DataCheckFreq"
                    ]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(
                    jsData["VisionSensors"][i]["CameraFOV"], float
                ) or isinstance(jsData["VisionSensors"][i]["CameraFOV"], int):
                    visSenStruct.CameraFOV = jsData["VisionSensors"][i]["CameraFOV"]
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["SendProtocol"]) == 8:
                    visSenStruct.SendProtocol = jsData["VisionSensors"][i][
                        "SendProtocol"
                    ]
                    if ChangeMode != -1:
                        # If in remote reception mode, image reading here needs to be configured for UDP reception
                        visSenStruct.SendProtocol[0] = ChangeMode
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["SensorPosXYZ"]) == 3:
                    visSenStruct.SensorPosXYZ = jsData["VisionSensors"][i][
                        "SensorPosXYZ"
                    ]
                else:
                    print("Json data format is wrong!")
                    continue

                isNewProt = False

                if "EularOrQuat" in jsData["VisionSensors"][i]:
                    isNewProt = True
                    visSenStruct.EularOrQuat = jsData["VisionSensors"][i]["EularOrQuat"]
                else:
                    visSenStruct.EularOrQuat = 0

                if len(jsData["VisionSensors"][i]["SensorAngEular"]) == 3:
                    visSenStruct.SensorAngEular = jsData["VisionSensors"][i][
                        "SensorAngEular"
                    ]
                else:
                    print("Json data format is wrong!")
                    continue

                if isNewProt:
                    if len(jsData["VisionSensors"][i]["SensorAngQuat"]) == 4:
                        visSenStruct.SensorAngQuat = jsData["VisionSensors"][i][
                            "SensorAngQuat"
                        ]
                    else:
                        print("Json data format is wrong!")
                        continue

                if isNewProt:  # New protocol uses 16-dimensional otherParams
                    if len(jsData["VisionSensors"][i]["otherParams"]) == 16:
                        visSenStruct.otherParams = jsData["VisionSensors"][i][
                            "otherParams"
                        ]
                    else:
                        print("Json data format is wrong!")
                        continue
                else:
                    if len(jsData["VisionSensors"][i]["otherParams"]) == 8:
                        visSenStruct.otherParams = (
                            jsData["VisionSensors"][i]["otherParams"] + [0] * 8
                        )  # Extend to 16 dimensions
                    else:
                        print("Json data format is wrong!")
                        continue
                self.VisSensor = self.VisSensor + [visSenStruct]

                if not self.isNewJson and isNewProt:
                    self.isNewJson = True

        if (len(self.VisSensor)) <= 0:
            print("No sensor is obtained.")
            return False
        print("Got", len(self.VisSensor), "vision sensors from json")

        if len(self.RflyTimeVect) == 0 and not self.tTimeStmpFlag:
            # print('Start listening CopterSim time Data')
            self.StartTimeStmplisten()
            time.sleep(2)
            self.endTimeStmplisten()
        if len(self.RflyTimeVect) > 0:
            print("Got CopterSim time Data for img")
        else:
            print("No CopterSim time Data for img")

        return True