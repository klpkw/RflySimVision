#!/bin/bash

# Start swarm script
cd /home/sensor_pkg
python3 single.py &
sleep 1

# Start Rflysim PX4 node
cd /home/ego-planner-swarm
source devel/setup.bash
roslaunch rflysim_px4_node rflysim_px4_single_udp.launch &
if [ $? -ne 0 ]; then
    echo "Failed to start rflysim_px4_single_udp"
    exit 1
fi

sleep 15

# Start control node
roslaunch px4ctrl run_ctrl_rflsyim_single.launch &
if [ $? -ne 0 ]; then
    echo "Failed to start run_ctrl_rflsyim_single"
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

sleep 5

# Start single drone simulation
roslaunch ego_planner rflysim_single_drone.launch
if [ $? -ne 0 ]; then
    echo "Failed to start rflysim_single_drone"
    exit 1
fi