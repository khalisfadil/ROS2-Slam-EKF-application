#!/bin/sh

xterm -e "cd $(pwd)/../result/autobin_result/output_05_aug/scene_7/kd_non_7;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 non --all_topics --save_as_tum;
evo_traj bag2 non --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_05_aug/scene_7/kd_non_7;
cp current_pose.tum tum/kd_non_7;
cp ref_pose.tum tum/kd_non_7_ref;
cp current_pose.kitti kitti/kd_non_7;
cp ref_pose.kitti kitti/kd_non_7_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_05_aug/scene_7/kd_non_7;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/kd_non_7;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/kd_non_7" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_05_aug/scene_7/kd_non_7;
evo_ape tum tum/kd_non_7_ref tum/kd_non_7 -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/kd_non_7_ape --save_results tum/distances/result.zip;
evo_ape tum tum/kd_non_7_ref tum/kd_non_7 -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/kd_non_7_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/kd_non_7_ref tum/kd_non_7 -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/kd_non_7_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/kd_non_7_ref kitti/kd_non_7 -v --plot --plot_mode xy --save_plot kitti/index/kd_non_7_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_05_aug/scene_7/kd_non_7;
evo_res tum/distances/result.zip -p --save_plot tum/distances/kd_non_7_res --ignore_title --use_rel_time --save_table tum/distances/kd_non_7.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/kd_non_7_res --ignore_title --use_rel_time --save_table tum/seconds/kd_non_7.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/kd_non_7_res --ignore_title --use_rel_time --save_table tum/index/kd_non_7.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/kd_non_7_res --ignore_title --use_rel_time --save_table kitti/index/kd_non_7.csv; " 