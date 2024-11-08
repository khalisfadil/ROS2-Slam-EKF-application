#!/bin/sh

xterm -e "cd $(pwd)/../dev2_ws;
source /opt/ros/foxy/setup.bash;
. install/setup.bash;
ros2 launch my_package slam.launch.py" &

sleep 5

xterm  -e  " cd $(pwd)/../dev2_ws/src/my_package/rviz;
source /opt/ros/foxy/setup.bash;
rviz2 -d mapping.rviz " &

sleep 5

xterm -e " cd /opt/my_data/v04_bag;
source /opt/ros/noetic/setup.bash;
source /opt/ros/foxy/setup.bash;
ros2 bag play -s rosbag_v2 30.bag --topics front_lidar /odom;
sleep 15;
ros2 bag play -s rosbag_v2 9.bag --topics front_lidar /odom;" &

sleep 182

xterm -e "./rosbagrecord.sh" &



