# import required libraries
# pip3 install pymavlink pyserial

import cv2
import numpy as np
import time
import VisionCaptureApi
import math

import ReqCopterSim
import RflyRosStart

import sys
# Enable ROS publishing mode

print(RflyRosStart.isLinux,RflyRosStart.isRosOk)
req = ReqCopterSim.ReqCopterSim()
VisionCaptureApi.isEnableRosTrans = True

StartCopterID = 1 # Initial aircraft ID number
TargetIP = req.getSimIpID(StartCopterID)
# Automatically enable mavros
# if not (RflyRosStart.isLinux and RflyRosStart.isRosOk):
#     print('This demo can only run on with Ros')
#     sys.exit(0)

# Automatically start RosCore
# ros = RflyRosStart.RflyRosStart(StartCopterID,TargetIP)

# Configuration function in VisionCaptureApi
vis = VisionCaptureApi.VisionCaptureApi()
vis.jsonLoad(jsonPath = "/home/sensor_pkg/singleConfig.json") # Load the sensor configuration file from Config.json
isSuss = vis.sendReqToUE4(
    0, "127.0.0.1"
)
vis.startImgCap()  # Start the image capture loop; after executing this statement, images can be read from vis.Img[i]
print('Start Image Reciver')

vis.sendImuReqCopterSim(StartCopterID,TargetIP)
