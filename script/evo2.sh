#!/bin/sh

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_BASELINE;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_BASELINE;
cp current_pose.tum tum/NDT_D1_BASELINE;
cp ref_pose.tum tum/NDT_D1_BASELINE_ref;
cp current_pose.kitti kitti/NDT_D1_BASELINE;
cp ref_pose.kitti kitti/NDT_D1_BASELINE_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_BASELINE;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_D1_BASELINE;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_D1_BASELINE" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_BASELINE;
evo_ape tum tum/NDT_D1_BASELINE_ref tum/NDT_D1_BASELINE -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_D1_BASELINE_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_BASELINE;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_D1_BASELINE_res --ignore_title --use_rel_time --save_table tum/distances/NDT_D1_BASELINE.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CA;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CA;
cp current_pose.tum tum/NDT_D1_CA;
cp ref_pose.tum tum/NDT_D1_CA_ref;
cp current_pose.kitti kitti/NDT_D1_CA;
cp ref_pose.kitti kitti/NDT_D1_CA_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CA;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_D1_CA;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_D1_CA" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CA;
evo_ape tum tum/NDT_D1_CA_ref tum/NDT_D1_CA -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_D1_CA_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CA;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_D1_CA_res --ignore_title --use_rel_time --save_table tum/distances/NDT_D1_CA.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CV;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CV;
cp current_pose.tum tum/NDT_D1_CV;
cp ref_pose.tum tum/NDT_D1_CV_ref;
cp current_pose.kitti kitti/NDT_D1_CV;
cp ref_pose.kitti kitti/NDT_D1_CV_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CV;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_D1_CV;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_D1_CV" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CV;
evo_ape tum tum/NDT_D1_CV_ref tum/NDT_D1_CV -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_D1_CV_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_CV;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_D1_CV_res --ignore_title --use_rel_time --save_table tum/distances/NDT_D1_CV.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_NON;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_NON;
cp current_pose.tum tum/NDT_D1_NON;
cp ref_pose.tum tum/NDT_D1_NON_ref;
cp current_pose.kitti kitti/NDT_D1_NON;
cp ref_pose.kitti kitti/NDT_D1_NON_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_NON;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_D1_NON;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_D1_NON" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_NON;
evo_ape tum tum/NDT_D1_NON_ref tum/NDT_D1_NON -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_D1_NON_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D1_NON;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_D1_NON_res --ignore_title --use_rel_time --save_table tum/distances/NDT_D1_NON.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_BASELINE;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_BASELINE;
cp current_pose.tum tum/NDT_D7_BASELINE;
cp ref_pose.tum tum/NDT_D7_BASELINE_ref;
cp current_pose.kitti kitti/NDT_D7_BASELINE;
cp ref_pose.kitti kitti/NDT_D7_BASELINE_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_BASELINE;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_D7_BASELINE;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_D7_BASELINE" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_BASELINE;
evo_ape tum tum/NDT_D7_BASELINE_ref tum/NDT_D7_BASELINE -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_D7_BASELINE_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_BASELINE;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_D7_BASELINE_res --ignore_title --use_rel_time --save_table tum/distances/NDT_D7_BASELINE.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CA;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CA;
cp current_pose.tum tum/NDT_D7_CA;
cp ref_pose.tum tum/NDT_D7_CA_ref;
cp current_pose.kitti kitti/NDT_D7_CA;
cp ref_pose.kitti kitti/NDT_D7_CA_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CA;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_D7_CA;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_D7_CA" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CA;
evo_ape tum tum/NDT_D7_CA_ref tum/NDT_D7_CA -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_D7_CA_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CA;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_D7_CA_res --ignore_title --use_rel_time --save_table tum/distances/NDT_D7_CA.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CV;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CV;
cp current_pose.tum tum/NDT_D7_CV;
cp ref_pose.tum tum/NDT_D7_CV_ref;
cp current_pose.kitti kitti/NDT_D7_CV;
cp ref_pose.kitti kitti/NDT_D7_CV_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CV;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_D7_CV;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_D7_CV" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CV;
evo_ape tum tum/NDT_D7_CV_ref tum/NDT_D7_CV -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_D7_CV_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_CV;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_D7_CV_res --ignore_title --use_rel_time --save_table tum/distances/NDT_D7_CV.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_NON;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_NON;
cp current_pose.tum tum/NDT_D7_NON;
cp ref_pose.tum tum/NDT_D7_NON_ref;
cp current_pose.kitti kitti/NDT_D7_NON;
cp ref_pose.kitti kitti/NDT_D7_NON_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_NON;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_D7_NON;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_D7_NON" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_NON;
evo_ape tum tum/NDT_D7_NON_ref tum/NDT_D7_NON -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_D7_NON_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_D7_NON;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_D7_NON_res --ignore_title --use_rel_time --save_table tum/distances/NDT_D7_NON.csv;
" 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_BASELINE;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_BASELINE;
cp current_pose.tum tum/NDT_KD_BASELINE;
cp ref_pose.tum tum/NDT_KD_BASELINE_ref;
cp current_pose.kitti kitti/NDT_KD_BASELINE;
cp ref_pose.kitti kitti/NDT_KD_BASELINE_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_BASELINE;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_KD_BASELINE;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_KD_BASELINE" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_BASELINE;
evo_ape tum tum/NDT_KD_BASELINE_ref tum/NDT_KD_BASELINE -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_KD_BASELINE_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_BASELINE;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_KD_BASELINE_res --ignore_title --use_rel_time --save_table tum/distances/NDT_KD_BASELINE.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CA;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CA;
cp current_pose.tum tum/NDT_KD_CA;
cp ref_pose.tum tum/NDT_KD_CA_ref;
cp current_pose.kitti kitti/NDT_KD_CA;
cp ref_pose.kitti kitti/NDT_KD_CA_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CA;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_KD_CA;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_KD_CA" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CA;
evo_ape tum tum/NDT_KD_CA_ref tum/NDT_KD_CA -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_KD_CA_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CA;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_KD_CA_res --ignore_title --use_rel_time --save_table tum/distances/NDT_KD_CA.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CV;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CV;
cp current_pose.tum tum/NDT_KD_CV;
cp ref_pose.tum tum/NDT_KD_CV_ref;
cp current_pose.kitti kitti/NDT_KD_CV;
cp ref_pose.kitti kitti/NDT_KD_CV_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CV;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_KD_CV;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_KD_CV" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CV;
evo_ape tum tum/NDT_KD_CV_ref tum/NDT_KD_CV -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_KD_CV_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_CV;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_KD_CV_res --ignore_title --use_rel_time --save_table tum/distances/NDT_KD_CV.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_NON;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_NON;
cp current_pose.tum tum/NDT_KD_NON;
cp ref_pose.tum tum/NDT_KD_NON_ref;
cp current_pose.kitti kitti/NDT_KD_NON;
cp ref_pose.kitti kitti/NDT_KD_NON_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_NON;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/NDT_KD_NON;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/NDT_KD_NON" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_NON;
evo_ape tum tum/NDT_KD_NON_ref tum/NDT_KD_NON -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/NDT_KD_NON_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/NDT_KD_NON;
evo_res tum/distances/result.zip -p --save_plot tum/distances/NDT_KD_NON_res --ignore_title --use_rel_time --save_table tum/distances/NDT_KD_NON.csv;
" 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_BASELINE;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_BASELINE;
cp current_pose.tum tum/ICP_KD_BASELINE;
cp ref_pose.tum tum/ICP_KD_BASELINE_ref;
cp current_pose.kitti kitti/ICP_KD_BASELINE;
cp ref_pose.kitti kitti/ICP_KD_BASELINE_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_BASELINE;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_KD_BASELINE;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_KD_BASELINE" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_BASELINE;
evo_ape tum tum/ICP_KD_BASELINE_ref tum/ICP_KD_BASELINE -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/ICP_KD_BASELINE_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_BASELINE;
evo_res tum/distances/result.zip -p --save_plot tum/distances/ICP_KD_BASELINE_res --ignore_title --use_rel_time --save_table tum/distances/ICP_KD_BASELINE.csv;
; " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CA;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CA;
cp current_pose.tum tum/ICP_KD_CA;
cp ref_pose.tum tum/ICP_KD_CA_ref;
cp current_pose.kitti kitti/ICP_KD_CA;
cp ref_pose.kitti kitti/ICP_KD_CA_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CA;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_KD_CA;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_KD_CA" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CA;
evo_ape tum tum/ICP_KD_CA_ref tum/ICP_KD_CA -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/ICP_KD_CA_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CA;
evo_res tum/distances/result.zip -p --save_plot tum/distances/ICP_KD_CA_res --ignore_title --use_rel_time --save_table tum/distances/ICP_KD_CA.csv;
" 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CV;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CV;
cp current_pose.tum tum/ICP_KD_CV;
cp ref_pose.tum tum/ICP_KD_CV_ref;
cp current_pose.kitti kitti/ICP_KD_CV;
cp ref_pose.kitti kitti/ICP_KD_CV_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CV;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_KD_CV;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_KD_CV" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CV;
evo_ape tum tum/ICP_KD_CV_ref tum/ICP_KD_CV -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/ICP_KD_CV_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_CV;
evo_res tum/distances/result.zip -p --save_plot tum/distances/ICP_KD_CV_res --ignore_title --use_rel_time --save_table tum/distances/ICP_KD_CV.csv;
 " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_NON;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_NON;
cp current_pose.tum tum/ICP_KD_NON;
cp ref_pose.tum tum/ICP_KD_NON_ref;
cp current_pose.kitti kitti/ICP_KD_NON;
cp ref_pose.kitti kitti/ICP_KD_NON_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_NON;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_KD_NON;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_KD_NON" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_NON;
evo_ape tum tum/ICP_KD_NON_ref tum/ICP_KD_NON -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/ICP_KD_NON_ape --save_results tum/distances/result.zip;
" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41/ICP_KD_NON;
evo_res tum/distances/result.zip -p --save_plot tum/distances/ICP_KD_NON_res --ignore_title --use_rel_time --save_table tum/distances/ICP_KD_NON.csv;
 " 



