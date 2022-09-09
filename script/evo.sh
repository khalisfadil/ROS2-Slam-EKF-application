#!/bin/sh

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_BASELINE;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_BASELINE;
cp current_pose.tum tum/D1_BASELINE;
cp ref_pose.tum tum/D1_BASELINE_ref;
cp current_pose.kitti kitti/D1_BASELINE;
cp ref_pose.kitti kitti/D1_BASELINE_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_BASELINE;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/D1_BASELINE;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/D1_BASELINE" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_BASELINE;
evo_ape tum tum/D1_BASELINE_ref tum/D1_BASELINE -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/D1_BASELINE_ape --save_results tum/distances/result.zip;
evo_ape tum tum/D1_BASELINE_ref tum/D1_BASELINE -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/D1_BASELINE_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/D1_BASELINE_ref tum/D1_BASELINE -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/D1_BASELINE_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/D1_BASELINE_ref kitti/D1_BASELINE -v --plot --plot_mode xy --save_plot kitti/index/D1_BASELINE_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_BASELINE;
evo_res tum/distances/result.zip -p --save_plot tum/distances/D1_BASELINE_res --ignore_title --use_rel_time --save_table tum/distances/D1_BASELINE.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/D1_BASELINE_res --ignore_title --use_rel_time --save_table tum/seconds/D1_BASELINE.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/D1_BASELINE_res --ignore_title --use_rel_time --save_table tum/index/D1_BASELINE.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/D1_BASELINE_res --ignore_title --use_rel_time --save_table kitti/index/D1_BASELINE.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCA;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCA;
cp current_pose.tum tum/D1_CHCA;
cp ref_pose.tum tum/D1_CHCA_ref;
cp current_pose.kitti kitti/D1_CHCA;
cp ref_pose.kitti kitti/D1_CHCA_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCA;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/D1_CHCA;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/D1_CHCA" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCA;
evo_ape tum tum/D1_CHCA_ref tum/D1_CHCA -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/D1_CHCA_ape --save_results tum/distances/result.zip;
evo_ape tum tum/D1_CHCA_ref tum/D1_CHCA -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/D1_CHCA_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/D1_CHCA_ref tum/D1_CHCA -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/D1_CHCA_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/D1_CHCA_ref kitti/D1_CHCA -v --plot --plot_mode xy --save_plot kitti/index/D1_CHCA_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCA;
evo_res tum/distances/result.zip -p --save_plot tum/distances/D1_CHCA_res --ignore_title --use_rel_time --save_table tum/distances/D1_CHCA.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/D1_CHCA_res --ignore_title --use_rel_time --save_table tum/seconds/D1_CHCA.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/D1_CHCA_res --ignore_title --use_rel_time --save_table tum/index/D1_CHCA.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/D1_CHCA_res --ignore_title --use_rel_time --save_table kitti/index/D1_CHCA.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCV;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCV;
cp current_pose.tum tum/D1_CHCV;
cp ref_pose.tum tum/D1_CHCV_ref;
cp current_pose.kitti kitti/D1_CHCV;
cp ref_pose.kitti kitti/D1_CHCV_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCV;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/D1_CHCV;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/D1_CHCV" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCV;
evo_ape tum tum/D1_CHCV_ref tum/D1_CHCV -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/D1_CHCV_ape --save_results tum/distances/result.zip;
evo_ape tum tum/D1_CHCV_ref tum/D1_CHCV -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/D1_CHCV_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/D1_CHCV_ref tum/D1_CHCV -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/D1_CHCV_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/D1_CHCV_ref kitti/D1_CHCV -v --plot --plot_mode xy --save_plot kitti/index/D1_CHCV_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_CHCV;
evo_res tum/distances/result.zip -p --save_plot tum/distances/D1_CHCV_res --ignore_title --use_rel_time --save_table tum/distances/D1_CHCV.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/D1_CHCV_res --ignore_title --use_rel_time --save_table tum/seconds/D1_CHCV.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/D1_CHCV_res --ignore_title --use_rel_time --save_table tum/index/D1_CHCV.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/D1_CHCV_res --ignore_title --use_rel_time --save_table kitti/index/D1_CHCV.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_NON;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_NON;
cp current_pose.tum tum/D1_NON;
cp ref_pose.tum tum/D1_NON_ref;
cp current_pose.kitti kitti/D1_NON;
cp ref_pose.kitti kitti/D1_NON_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_NON;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/D1_NON;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/D1_NON" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_NON;
evo_ape tum tum/D1_NON_ref tum/D1_NON -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/D1_NON_ape --save_results tum/distances/result.zip;
evo_ape tum tum/D1_NON_ref tum/D1_NON -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/D1_NON_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/D1_NON_ref tum/D1_NON -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/D1_NON_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/D1_NON_ref kitti/D1_NON -v --plot --plot_mode xy --save_plot kitti/index/D1_NON_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D1_NON;
evo_res tum/distances/result.zip -p --save_plot tum/distances/D1_NON_res --ignore_title --use_rel_time --save_table tum/distances/D1_NON.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/D1_NON_res --ignore_title --use_rel_time --save_table tum/seconds/D1_NON.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/D1_NON_res --ignore_title --use_rel_time --save_table tum/index/D1_NON.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/D1_NON_res --ignore_title --use_rel_time --save_table kitti/index/D1_NON.csv; " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_BASELINE;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_BASELINE;
cp current_pose.tum tum/D7_BASELINE;
cp ref_pose.tum tum/D7_BASELINE_ref;
cp current_pose.kitti kitti/D7_BASELINE;
cp ref_pose.kitti kitti/D7_BASELINE_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_BASELINE;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/D7_BASELINE;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/D7_BASELINE" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_BASELINE;
evo_ape tum tum/D7_BASELINE_ref tum/D7_BASELINE -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/D7_BASELINE_ape --save_results tum/distances/result.zip;
evo_ape tum tum/D7_BASELINE_ref tum/D7_BASELINE -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/D7_BASELINE_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/D7_BASELINE_ref tum/D7_BASELINE -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/D7_BASELINE_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/D7_BASELINE_ref kitti/D7_BASELINE -v --plot --plot_mode xy --save_plot kitti/index/D7_BASELINE_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_BASELINE;
evo_res tum/distances/result.zip -p --save_plot tum/distances/D7_BASELINE_res --ignore_title --use_rel_time --save_table tum/distances/D7_BASELINE.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/D7_BASELINE_res --ignore_title --use_rel_time --save_table tum/seconds/D7_BASELINE.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/D7_BASELINE_res --ignore_title --use_rel_time --save_table tum/index/D7_BASELINE.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/D7_BASELINE_res --ignore_title --use_rel_time --save_table kitti/index/D7_BASELINE.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCA;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCA;
cp current_pose.tum tum/D7_CHCA;
cp ref_pose.tum tum/D7_CHCA_ref;
cp current_pose.kitti kitti/D7_CHCA;
cp ref_pose.kitti kitti/D7_CHCA_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCA;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/D7_CHCA;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/D7_CHCA" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCA;
evo_ape tum tum/D7_CHCA_ref tum/D7_CHCA -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/D7_CHCA_ape --save_results tum/distances/result.zip;
evo_ape tum tum/D7_CHCA_ref tum/D7_CHCA -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/D7_CHCA_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/D7_CHCA_ref tum/D7_CHCA -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/D7_CHCA_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/D7_CHCA_ref kitti/D7_CHCA -v --plot --plot_mode xy --save_plot kitti/index/D7_CHCA_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCA;
evo_res tum/distances/result.zip -p --save_plot tum/distances/D7_CHCA_res --ignore_title --use_rel_time --save_table tum/distances/D7_CHCA.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/D7_CHCA_res --ignore_title --use_rel_time --save_table tum/seconds/D7_CHCA.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/D7_CHCA_res --ignore_title --use_rel_time --save_table tum/index/D7_CHCA.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/D7_CHCA_res --ignore_title --use_rel_time --save_table kitti/index/D7_CHCA.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCV;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCV;
cp current_pose.tum tum/D7_CHCV;
cp ref_pose.tum tum/D7_CHCV_ref;
cp current_pose.kitti kitti/D7_CHCV;
cp ref_pose.kitti kitti/D7_CHCV_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCV;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/D7_CHCV;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/D7_CHCV" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCV;
evo_ape tum tum/D7_CHCV_ref tum/D7_CHCV -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/D7_CHCV_ape --save_results tum/distances/result.zip;
evo_ape tum tum/D7_CHCV_ref tum/D7_CHCV -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/D7_CHCV_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/D7_CHCV_ref tum/D7_CHCV -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/D7_CHCV_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/D7_CHCV_ref kitti/D7_CHCV -v --plot --plot_mode xy --save_plot kitti/index/D7_CHCV_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_CHCV;
evo_res tum/distances/result.zip -p --save_plot tum/distances/D7_CHCV_res --ignore_title --use_rel_time --save_table tum/distances/D7_CHCV.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/D7_CHCV_res --ignore_title --use_rel_time --save_table tum/seconds/D7_CHCV.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/D7_CHCV_res --ignore_title --use_rel_time --save_table tum/index/D7_CHCV.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/D7_CHCV_res --ignore_title --use_rel_time --save_table kitti/index/D7_CHCV.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_NON;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_NON;
cp current_pose.tum tum/D7_NON;
cp ref_pose.tum tum/D7_NON_ref;
cp current_pose.kitti kitti/D7_NON;
cp ref_pose.kitti kitti/D7_NON_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_NON;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/D7_NON;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/D7_NON" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_NON;
evo_ape tum tum/D7_NON_ref tum/D7_NON -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/D7_NON_ape --save_results tum/distances/result.zip;
evo_ape tum tum/D7_NON_ref tum/D7_NON -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/D7_NON_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/D7_NON_ref tum/D7_NON -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/D7_NON_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/D7_NON_ref kitti/D7_NON -v --plot --plot_mode xy --save_plot kitti/index/D7_NON_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/D7_NON;
evo_res tum/distances/result.zip -p --save_plot tum/distances/D7_NON_res --ignore_title --use_rel_time --save_table tum/distances/D7_NON.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/D7_NON_res --ignore_title --use_rel_time --save_table tum/seconds/D7_NON.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/D7_NON_res --ignore_title --use_rel_time --save_table tum/index/D7_NON.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/D7_NON_res --ignore_title --use_rel_time --save_table kitti/index/D7_NON.csv; " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_BASELINE;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_BASELINE;
cp current_pose.tum tum/KD_BASELINE;
cp ref_pose.tum tum/KD_BASELINE_ref;
cp current_pose.kitti kitti/KD_BASELINE;
cp ref_pose.kitti kitti/KD_BASELINE_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_BASELINE;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/KD_BASELINE;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/KD_BASELINE" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_BASELINE;
evo_ape tum tum/KD_BASELINE_ref tum/KD_BASELINE -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/KD_BASELINE_ape --save_results tum/distances/result.zip;
evo_ape tum tum/KD_BASELINE_ref tum/KD_BASELINE -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/KD_BASELINE_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/KD_BASELINE_ref tum/KD_BASELINE -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/KD_BASELINE_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/KD_BASELINE_ref kitti/KD_BASELINE -v --plot --plot_mode xy --save_plot kitti/index/KD_BASELINE_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_BASELINE;
evo_res tum/distances/result.zip -p --save_plot tum/distances/KD_BASELINE_res --ignore_title --use_rel_time --save_table tum/distances/KD_BASELINE.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/KD_BASELINE_res --ignore_title --use_rel_time --save_table tum/seconds/KD_BASELINE.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/KD_BASELINE_res --ignore_title --use_rel_time --save_table tum/index/KD_BASELINE.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/KD_BASELINE_res --ignore_title --use_rel_time --save_table kitti/index/KD_BASELINE.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCA;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCA;
cp current_pose.tum tum/KD_CHCA;
cp ref_pose.tum tum/KD_CHCA_ref;
cp current_pose.kitti kitti/KD_CHCA;
cp ref_pose.kitti kitti/KD_CHCA_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCA;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/KD_CHCA;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/KD_CHCA" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCA;
evo_ape tum tum/KD_CHCA_ref tum/KD_CHCA -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/KD_CHCA_ape --save_results tum/distances/result.zip;
evo_ape tum tum/KD_CHCA_ref tum/KD_CHCA -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/KD_CHCA_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/KD_CHCA_ref tum/KD_CHCA -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/KD_CHCA_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/KD_CHCA_ref kitti/KD_CHCA -v --plot --plot_mode xy --save_plot kitti/index/KD_CHCA_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCA;
evo_res tum/distances/result.zip -p --save_plot tum/distances/KD_CHCA_res --ignore_title --use_rel_time --save_table tum/distances/KD_CHCA.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/KD_CHCA_res --ignore_title --use_rel_time --save_table tum/seconds/KD_CHCA.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/KD_CHCA_res --ignore_title --use_rel_time --save_table tum/index/KD_CHCA.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/KD_CHCA_res --ignore_title --use_rel_time --save_table kitti/index/KD_CHCA.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCV;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCV;
cp current_pose.tum tum/KD_CHCV;
cp ref_pose.tum tum/KD_CHCV_ref;
cp current_pose.kitti kitti/KD_CHCV;
cp ref_pose.kitti kitti/KD_CHCV_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCV;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/KD_CHCV;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/KD_CHCV" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCV;
evo_ape tum tum/KD_CHCV_ref tum/KD_CHCV -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/KD_CHCV_ape --save_results tum/distances/result.zip;
evo_ape tum tum/KD_CHCV_ref tum/KD_CHCV -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/KD_CHCV_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/KD_CHCV_ref tum/KD_CHCV -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/KD_CHCV_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/KD_CHCV_ref kitti/KD_CHCV -v --plot --plot_mode xy --save_plot kitti/index/KD_CHCV_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_CHCV;
evo_res tum/distances/result.zip -p --save_plot tum/distances/KD_CHCV_res --ignore_title --use_rel_time --save_table tum/distances/KD_CHCV.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/KD_CHCV_res --ignore_title --use_rel_time --save_table tum/seconds/KD_CHCV.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/KD_CHCV_res --ignore_title --use_rel_time --save_table tum/index/KD_CHCV.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/KD_CHCV_res --ignore_title --use_rel_time --save_table kitti/index/KD_CHCV.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_NON;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_NON;
cp current_pose.tum tum/KD_NON;
cp ref_pose.tum tum/KD_NON_ref;
cp current_pose.kitti kitti/KD_NON;
cp ref_pose.kitti kitti/KD_NON_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_NON;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/KD_NON;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/KD_NON" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_NON;
evo_ape tum tum/KD_NON_ref tum/KD_NON -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/KD_NON_ape --save_results tum/distances/result.zip;
evo_ape tum tum/KD_NON_ref tum/KD_NON -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/KD_NON_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/KD_NON_ref tum/KD_NON -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/KD_NON_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/KD_NON_ref kitti/KD_NON -v --plot --plot_mode xy --save_plot kitti/index/KD_NON_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/KD_NON;
evo_res tum/distances/result.zip -p --save_plot tum/distances/KD_NON_res --ignore_title --use_rel_time --save_table tum/distances/KD_NON.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/KD_NON_res --ignore_title --use_rel_time --save_table tum/seconds/KD_NON.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/KD_NON_res --ignore_title --use_rel_time --save_table tum/index/KD_NON.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/KD_NON_res --ignore_title --use_rel_time --save_table kitti/index/KD_NON.csv; " 

sleep 1

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_BASELINE;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_BASELINE;
cp current_pose.tum tum/ICP_BASELINE;
cp ref_pose.tum tum/ICP_BASELINE_ref;
cp current_pose.kitti kitti/ICP_BASELINE;
cp ref_pose.kitti kitti/ICP_BASELINE_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_BASELINE;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_BASELINE;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_BASELINE" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_BASELINE;
evo_ape tum tum/ICP_BASELINE_ref tum/ICP_BASELINE -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/ICP_BASELINE_ape --save_results tum/distances/result.zip;
evo_ape tum tum/ICP_BASELINE_ref tum/ICP_BASELINE -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/ICP_BASELINE_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/ICP_BASELINE_ref tum/ICP_BASELINE -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/ICP_BASELINE_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/ICP_BASELINE_ref kitti/ICP_BASELINE -v --plot --plot_mode xy --save_plot kitti/index/ICP_BASELINE_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_BASELINE;
evo_res tum/distances/result.zip -p --save_plot tum/distances/ICP_BASELINE_res --ignore_title --use_rel_time --save_table tum/distances/ICP_BASELINE.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/ICP_BASELINE_res --ignore_title --use_rel_time --save_table tum/seconds/ICP_BASELINE.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/ICP_BASELINE_res --ignore_title --use_rel_time --save_table tum/index/ICP_BASELINE.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/ICP_BASELINE_res --ignore_title --use_rel_time --save_table kitti/index/ICP_BASELINE.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCA;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCA;
cp current_pose.tum tum/ICP_CHCA;
cp ref_pose.tum tum/ICP_CHCA_ref;
cp current_pose.kitti kitti/ICP_CHCA;
cp ref_pose.kitti kitti/ICP_CHCA_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCA;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_CHCA;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_CHCA" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCA;
evo_ape tum tum/ICP_CHCA_ref tum/ICP_CHCA -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/ICP_CHCA_ape --save_results tum/distances/result.zip;
evo_ape tum tum/ICP_CHCA_ref tum/ICP_CHCA -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/ICP_CHCA_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/ICP_CHCA_ref tum/ICP_CHCA -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/ICP_CHCA_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/ICP_CHCA_ref kitti/ICP_CHCA -v --plot --plot_mode xy --save_plot kitti/index/ICP_CHCA_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCA;
evo_res tum/distances/result.zip -p --save_plot tum/distances/ICP_CHCA_res --ignore_title --use_rel_time --save_table tum/distances/ICP_CHCA.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/ICP_CHCA_res --ignore_title --use_rel_time --save_table tum/seconds/ICP_CHCA.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/ICP_CHCA_res --ignore_title --use_rel_time --save_table tum/index/ICP_CHCA.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/ICP_CHCA_res --ignore_title --use_rel_time --save_table kitti/index/ICP_CHCA.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCV;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCV;
cp current_pose.tum tum/ICP_CHCV;
cp ref_pose.tum tum/ICP_CHCV_ref;
cp current_pose.kitti kitti/ICP_CHCV;
cp ref_pose.kitti kitti/ICP_CHCV_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCV;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_CHCV;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_CHCV" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCV;
evo_ape tum tum/ICP_CHCV_ref tum/ICP_CHCV -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/ICP_CHCV_ape --save_results tum/distances/result.zip;
evo_ape tum tum/ICP_CHCV_ref tum/ICP_CHCV -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/ICP_CHCV_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/ICP_CHCV_ref tum/ICP_CHCV -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/ICP_CHCV_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/ICP_CHCV_ref kitti/ICP_CHCV -v --plot --plot_mode xy --save_plot kitti/index/ICP_CHCV_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_CHCV;
evo_res tum/distances/result.zip -p --save_plot tum/distances/ICP_CHCV_res --ignore_title --use_rel_time --save_table tum/distances/ICP_CHCV.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/ICP_CHCV_res --ignore_title --use_rel_time --save_table tum/seconds/ICP_CHCV.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/ICP_CHCV_res --ignore_title --use_rel_time --save_table tum/index/ICP_CHCV.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/ICP_CHCV_res --ignore_title --use_rel_time --save_table kitti/index/ICP_CHCV.csv; " 

sleep 2

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_NON;
mkdir -p tum/distances;
mkdir -p tum/seconds;
mkdir -p tum/index;
mkdir -p kitti/index;
evo_traj bag2 rosbag --all_topics --save_as_tum;
evo_traj bag2 rosbag --all_topics --save_as_kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_NON;
cp current_pose.tum tum/ICP_NON;
cp ref_pose.tum tum/ICP_NON_ref;
cp current_pose.kitti kitti/ICP_NON;
cp ref_pose.kitti kitti/ICP_NON_ref; 
mv current_pose.tum tum/estimated_pose.tum;
mv ref_pose.tum tum/reference.tum;
mv current_pose.kitti kitti/estimated_pose.kitti;
mv ref_pose.kitti kitti/reference.kitti" &

sleep 5

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_NON;
evo_traj tum tum/estimated_pose.tum --ref tum/reference.tum -p --plot_mode=xy  --save_plot tum/ICP_NON;
evo_traj kitti kitti/estimated_pose.kitti --ref kitti/reference.kitti -p --plot_mode=xy  --save_plot kitti/ICP_NON" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_NON;
evo_ape tum tum/ICP_NON_ref tum/ICP_NON -v --plot --plot_mode xy --plot_x_dimension distances --save_plot tum/distances/ICP_NON_ape --save_results tum/distances/result.zip;
evo_ape tum tum/ICP_NON_ref tum/ICP_NON -v --plot --plot_mode xy --plot_x_dimension seconds --save_plot tum/seconds/ICP_NON_ape --save_results tum/seconds/result.zip;
evo_ape tum tum/ICP_NON_ref tum/ICP_NON -v --plot --plot_mode xy --plot_x_dimension index --save_plot tum/index/ICP_NON_ape --save_results tum/index/result.zip;
evo_ape kitti kitti/ICP_NON_ref kitti/ICP_NON -v --plot --plot_mode xy --save_plot kitti/index/ICP_NON_ape --save_results kitti/index/result.zip" &&

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_34/ICP_NON;
evo_res tum/distances/result.zip -p --save_plot tum/distances/ICP_NON_res --ignore_title --use_rel_time --save_table tum/distances/ICP_NON.csv;
evo_res tum/seconds/result.zip -p --save_plot tum/seconds/ICP_NON_res --ignore_title --use_rel_time --save_table tum/seconds/ICP_NON.csv;
evo_res tum/index/result.zip -p --save_plot tum/index/ICP_NON_res --ignore_title --use_rel_time --save_table tum/index/ICP_NON.csv;
evo_res kitti/index/result.zip -p --save_plot kitti/index/ICP_NON_res --ignore_title --use_rel_time --save_table kitti/index/ICP_NON.csv; " 



