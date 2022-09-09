#!/bin/sh

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_2/scene_9/NDT_KD_NEGLECT;
source /opt/ros/foxy/setup.bash;
ros2 bag record -o rosbag /current_pose /ref_pose" 

#xterm -e "cd $(pwd)/../result/autobin_result/output_01_aug/d7_chca_7;
#source /opt/ros/foxy/setup.bash;
#ros2 topic echo /current_pose --csv > current_pose.csv" &

#xterm -e "cd $(pwd)/../result/autobin_result/output_01_aug/d7_chca_7;
#source /opt/ros/foxy/setup.bash;
#ros2 topic echo /ref_pose --csv > ref_pose.csv" &

#xterm -e "cd $(pwd)/../dev2_ws;
#source /opt/ros/foxy/setup.bash;
#. install/setup.bash;
#cd $(pwd)/../result/autobin_result/output_01_aug/d7_chca_7;
#ros2 topic echo /kalman_chca --csv > kalman_pose.csv"