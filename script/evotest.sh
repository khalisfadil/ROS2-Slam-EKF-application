#!/bin/sh

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_2/scene_7/ICP_KD_NEGLECT;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_2/scene_7/ICP_KD_NEGLECT;
cp current_pose.tum tum/ICP_KD_NEGLECT;
cp ref_pose.tum tum/N_KD_NEGLECT_ref;
cp current_pose.kitti kitti/ICP_KD_NEGLECT;
cp ref_pose.kitti kitti/N_KD_NEGLECT_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_2/scene_7/ICP_KD_NEGLECT;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_KD_NEGLECT;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_KD_NEGLECT" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_2/scene_7/ICP_KD_NEGLECT;
evo_ape tum tum/N_KD_NEGLECT_ref tum/ICP_KD_NEGLECT -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/N_KD_NEGLECT_ape --save_results tum/distances/result.zip;
evo_ape kitti kitti/N_KD_NEGLECT_ref kitti/ICP_KD_NEGLECT -v --plot --plot_mode xy --save_plot kitti/index/N_KD_NEGLECT_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_2/scene_7/ICP_KD_NEGLECT;
evo_res tum/distances/result.zip -p --save_plot tum/distances/N_KD_NEGLECT_res --ignore_title --use_rel_time --save_table tum/distances/ICP_KD_NEGLECT.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/N_KD_NEGLECT_res --ignore_title --use_rel_time --save_table kitti/index/ICP_KD_NEGLECT.csv; " 