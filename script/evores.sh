#!/bin/sh

xterm -e "cd $(pwd)/../result/autobin_result/output_05_aug/scene_7;
mkdir -p result/d1_7_res;
evo_res d1_chca_7/tum/distances/result.zip d1_chcv_7/tum/distances/result.zip d1_non_7/tum/distances/result.zip -p --save_plot result/d1_7_res/d1_7_res --ignore_title --use_rel_time --save_table result/d1_7_res/d1_7_res.csv;
mkdir -p result/d7_7_res;
evo_res d7_chca_7/tum/distances/result.zip d7_chcv_7/tum/distances/result.zip d7_non_7/tum/distances/result.zip -p --save_plot result/d7_7_res/d7_7_res --ignore_title --use_rel_time --save_table result/d7_7_res/d7_7_res.csv;
mkdir -p result/kd_7_res;
evo_res kd_chca_7/tum/distances/result.zip kd_chcv_7/tum/distances/result.zip kd_non_7/tum/distances/result.zip -p --save_plot result/kd_7_res/kd_7_res --ignore_title --use_rel_time --save_table result/kd_7_res/kd_7_res.csv;
mkdir -p result/chca_7_res;
evo_res d1_chca_7/tum/distances/result.zip d7_chca_7/tum/distances/result.zip kd_chca_7/tum/distances/result.zip -p --save_plot result/chca_7_res/chca_7_res --ignore_title --use_rel_time --save_table result/chca_7_res/chca_7_res.csv;
mkdir -p result/chcv_7_res;
evo_res d1_chcv_7/tum/distances/result.zip d7_chcv_7/tum/distances/result.zip kd_chcv_7/tum/distances/result.zip -p --save_plot result/chcv_7_res/chcv_7_res --ignore_title --use_rel_time --save_table result/chcv_7_res/chcv_7_res.csv;
mkdir -p result/non_7_res;
evo_res d1_non_7/tum/distances/result.zip d7_non_7/tum/distances/result.zip kd_non_7/tum/distances/result.zip -p --save_plot result/non_7_res/non_7_res --ignore_title --use_rel_time --save_table result/non_7_res/non_7_res.csv;

mkdir -p result/d1_7_index_res;
evo_res d1_chca_7/tum/index/result.zip d1_chcv_7/tum/index/result.zip d1_non_7/tum/index/result.zip -p --save_plot result/d1_7_index_res/d1_7_res --ignore_title --use_rel_time --save_table result/d1_7_index_res/d1_7_res.csv;
mkdir -p result/d7_7_index_res;
evo_res d7_chca_7/tum/index/result.zip d7_chcv_7/tum/index/result.zip d7_non_7/tum/index/result.zip -p --save_plot result/d7_7_index_res/d7_7_res --ignore_title --use_rel_time --save_table result/d7_7_index_res/d7_7_res.csv;
mkdir -p result/kd_7_index_res;
evo_res kd_chca_7/tum/index/result.zip kd_chcv_7/tum/index/result.zip kd_non_7/tum/index/result.zip -p --save_plot result/kd_7_index_res/kd_7_res --ignore_title --use_rel_time --save_table result/kd_7_index_res/kd_7_res.csv;
mkdir -p result/chca_7_index_res;
evo_res d1_chca_7/tum/index/result.zip d7_chca_7/tum/index/result.zip kd_chca_7/tum/index/result.zip -p --save_plot result/chca_7_index_res/chca_7_res --ignore_title --use_rel_time --save_table result/chca_7_index_res/chca_7_res.csv;
mkdir -p result/chcv_7_index_res;
evo_res d1_chcv_7/tum/index/result.zip d7_chcv_7/tum/index/result.zip kd_chcv_7/tum/index/result.zip -p --save_plot result/chcv_7_index_res/chcv_7_res --ignore_title --use_rel_time --save_table result/chcv_7_index_res/chcv_7_res.csv;
mkdir -p result/non_7_index_res;
evo_res d1_non_7/tum/index/result.zip d7_non_7/tum/index/result.zip kd_non_7/tum/index/result.zip -p --save_plot result/non_7_index_res/non_7_res --ignore_title --use_rel_time --save_table result/non_7_index_res/non_7_res.csv " 