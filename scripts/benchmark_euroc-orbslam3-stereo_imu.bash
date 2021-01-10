#!/bin/bash
set -e
SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
EUROC_DIR=/data/euroc_mav/
ORBSLAM3_DIR=src/ORB_SLAM3
DATASETS=(MH_01 MH_02 MH_03 MH_04 MH_05 V1_01 V1_02 V1_03 V2_01 V2_02 V2_03)
RUNS=(run1 run2 run3 run4 run5 run6 run7 run8 run9 run10)

# AUTOCAL
CALIB_FILE=$SCRIPT_PATH/../configs/autocal/orbslam3/euroc-stereo_imu.yaml
RES_DIR=/data/results/euroc/orbslam3-stereo_imu-autocal

# KALIBR
# CALIB_FILE=$SCRIPT_PATH/../configs/kalibr/euroc-stereo_imu.yaml
# RES_DIR=/data/results/euroc/orbslam3-stereo_imu-kalibr

# # EUROC
# CALIB_FILE=$SCRIPT_PATH/../configs/orbslam3/euroc-stereo_imu.yaml
# RES_DIR=/data/results/euroc/orbslam3-stereo_imu-euroc

for RUN in ${RUNS[@]}; do
  echo $RUN
  mkdir -p $RES_DIR/$RUN

  for DS in ${DATASETS[@]}; do
    ./${ORBSLAM3_DIR}/Examples/Stereo-Inertial/stereo_inertial_euroc \
      ${ORBSLAM3_DIR}/Vocabulary/ORBvoc.txt \
      ${CALIB_FILE} \
      "$EUROC_DIR"/${DS} \
      ${ORBSLAM3_DIR}/Examples/Stereo-Inertial/EuRoC_TimeStamps/${DS/_/}.txt \
      dataset-${DS/_/}_stereo_imu
  done

  mv f_dataset-*_stereo_imu.txt $RES_DIR/$RUN
  mv kf_dataset-*_stereo_imu.txt $RES_DIR/$RUN
done
