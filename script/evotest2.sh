#!/bin/sh

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_2/scene_41;
mkdir -p RESULT/FIRST;
evo_res NDT_KD_BASELINE/tum/distances/result.zip ICP_KD_BASELINE/tum/distances/result.zip  -p --save_plot RESULT/FIRST/scene_41 --ignore_title --use_rel_time --save_table RESULT/FIRST/scene_41.csv;"&
