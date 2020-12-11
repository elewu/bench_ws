#!/bin/bash
set -e

# make
source devel/setup.bash
source scripts/benchmark.bash

python scripts/compute_stereo_rectify.py \
   --calib_file /tmp/calib_stereo.yaml \
   --calib_format autocal \
   --mode VO \
   > src/bench/configs/autocal/orbslam3/euroc-stereo.yaml

# python scripts/compute_stereo_rectify.py \
#    --calib_file src/bench/configs/kalibr/camchain.yaml \
#    --calib_format kalibr \
#    --mode VO \
#    > src/bench/configs/kalibr/euroc-stereo.yaml


# ./scripts/benchmark_euroc-orbslam3.bash

# RESULTS_DIR=/data/results/euroc
# ANALYSIS_DIR=/data/results/euroc/analysis_vo
# ANALYSIS_YAML=/data/results/euroc/euroc_vo_analysis.yaml
# analyze_euroc_runs $RESULTS_DIR $ANALYSIS_DIR $ANALYSIS_YAML

# rosrun \
#   rpg_trajectory_evaluation \
#   analyze_trajectory_single.py \
#   /tmp/laptop_orbslam3-autocal_MH_01


SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
DATASETS=(MH_01 MH_02 MH_03 MH_04 MH_05 V1_01 V1_02 V1_03 V2_01 V2_02 V2_03)

CALIB_FILE=$PWD/src/bench/configs/autocal/orbslam3/euroc-stereo.yaml
RES_DIR=/data/results/euroc/orbslam3-autocal
RES_FILE=./results-autocal

# CALIB_FILE=$PWD/src/bench/configs/kalibr/euroc-stereo.yaml
# RES_DIR=/data/results/euroc/orbslam3-kalibr
# RES_FILE=./results-kalibr

# CALIB_FILE=$PWD/src/bench/configs/orbslam3/euroc-stereo.yaml
# RES_DIR=/data/results/euroc/orbslam3-euroc
# RES_FILE=./results-euroc

ORBSLAM3_DIR=src/ORB_SLAM3
EUROC_DIR=/data/euroc_mav/

RUNS=(run1 run2 run3 run4 run5 run6 run7 run8 run9 run10)

for RUN in ${RUNS[@]}; do
  echo $RUN

  for DS in ${DATASETS[@]}; do
    ./${ORBSLAM3_DIR}/Examples/Stereo/stereo_euroc \
      ${ORBSLAM3_DIR}/Vocabulary/ORBvoc.txt \
      ${CALIB_FILE} \
      "$EUROC_DIR"/${DS} \
      ${ORBSLAM3_DIR}/Examples/Stereo/EuRoC_TimeStamps/${DS/_/}.txt \
      dataset-${DS/_/}_stereo
  done

  mkdir -p $RES_DIR/$RUN
  mv f_dataset-*_stereo.txt $RES_DIR/$RUN
  mv kf_dataset-*_stereo.txt $RES_DIR/$RUN
done

# ORB_SLAM3_DIR=$SCRIPT_PATH/../src/ORB_SLAM3
# EVAL_SCRIPT=$ORB_SLAM3_DIR/evaluation/evaluate_ate_scale.py
#
# for DS in ${DATASETS[@]}; do
#   echo $DS >> ${RES_FILE}-${DS}.txt
#
#   for RUN in ${RUNS[@]}; do
#     python $EVAL_SCRIPT \
#       $ORB_SLAM3_DIR/evaluation/Ground_truth/EuRoC_left_cam/${DS/_/}_GT.txt \
#       $RES_DIR/$RUN/f_dataset-${DS/_/}_stereo.txt >> ${RES_FILE}-${DS}.txt
#   done
#
#   echo "" >> ${RES_FILE}-${DS}.txt
# done

# ./euroc_eval.sh > results_euroc.txt
# ./euroc_eval.sh > results_autocal.txt
