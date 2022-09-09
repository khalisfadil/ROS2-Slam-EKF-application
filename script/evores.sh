#!/bin/sh

xterm -e "cd $(pwd)/../result/autobin_result/output_18_aug/set_1/scene_41;
mkdir -p RESULT/D1;
evo_res NDT_D1_BASELINE/tum/distances/result.zip NDT_D1_CA/tum/distances/result.zip NDT_D1_CV/tum/distances/result.zip NDT_D1_NON/tum/distances/result.zip -p --save_plot RESULT/D1/D1 --ignore_title --use_rel_time --save_table RESULT/D1/D1.csv;
mkdir -p RESULT/D7;
evo_res NDT_D7_BASELINE/tum/distances/result.zip NDT_D7_CA/tum/distances/result.zip NDT_D7_CV/tum/distances/result.zip NDT_D7_NON/tum/distances/result.zip -p --save_plot RESULT/D7/D7 --ignore_title --use_rel_time --save_table RESULT/D7/D7.csv;
mkdir -p RESULT/KD;
evo_res NDT_KD_BASELINE/tum/distances/result.zip NDT_KD_CA/tum/distances/result.zip NDT_KD_CV/tum/distances/result.zip NDT_KD_NON/tum/distances/result.zip -p --save_plot RESULT/KD/KD --ignore_title --use_rel_time --save_table RESULT/KD/KD.csv;
mkdir -p RESULT/ICP;
evo_res ICP_KD__BASELINE/tum/distances/result.zip ICP_KD_CA/tum/distances/result.zip ICP_KD_CV/tum/distances/result.zip ICP_KD_NON/tum/distances/result.zip -p --save_plot RESULT/ICP/ICP --ignore_title --use_rel_time --save_table RESULT/ICP/ICP.csv;

mkdir -p RESULT/CA;
evo_res NDT_D1_CA/tum/distances/result.zip NDT_D7_CA/tum/distances/result.zip NDT_KD_CA/tum/distances/result.zip ICP_KD_CA/tum/distances/result.zip -p --save_plot RESULT/CA/CA --ignore_title --use_rel_time --save_table RESULT/CA/CA.csv;
mkdir -p RESULT/CV;
evo_res NDT_D1_CV/tum/distances/result.zip NDT_D7_CV/tum/distances/result.zip NDT_KD_CV/tum/distances/result.zip ICP_KD_CV/tum/distances/result.zip -p --save_plot RESULT/CV/CV --ignore_title --use_rel_time --save_table RESULT/CV/CV.csv;

"&


