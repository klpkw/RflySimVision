#!/bin/bash

# Start swarm script
cd /home/sensor_pkg
python swarm.py &
sleep 1

# Start Rflysim PX4 node
cd /home/ego-planner-swarm
source devel/setup.bash
roslaunch rflysim_px4_node rflysim_px4_three_udp.launch &
if [ $? -ne 0 ]; then
    echo "Failed to start rflysim_px4_three_udp"
    exit 1
fi

sleep 15

# Start control node
roslaunch px4ctrl run_ctrl_rflsyim_three.launch &
if [ $? -ne 0 ]; then
    echo "Failed to start run_ctrl_rflsyim_three"
    exit 1
fi

sleep 3

# Start RViz
roslaunch ego_planner rviz.launch &
if [ $? -ne 0 ]; then
    echo "Failed to start RViz"
    exit 1
fi

sleep 10

# Takeoff command
rostopic pub -1 /px4ctrl_uav0/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
rostopic pub -1 /px4ctrl_uav1/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
rostopic pub -1 /px4ctrl_uav2/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"

sleep 5

# Start simulation for three drones
roslaunch ego_planner rflysim_three_drone.launch
if [ $? -ne 0 ]; then
    echo "Failed to start rflysim_three_drone"
    exit 1
fi
