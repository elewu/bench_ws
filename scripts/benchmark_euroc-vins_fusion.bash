#!/bin/bash
set -e
SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
source $SCRIPT_PATH/benchmark.bash
source $SCRIPT_PATH/../devel/setup.bash
trap 'kill -9 %1' 2

# VINS-Fusion
LAUNCH_FILE=benchmark_euroc-vins_fusion.launch
CONFIG_FILE=$SCRIPT_PATH/../configs/autocal/vins_fusion/euroc-stereo_imu.yaml
RESULTS_DIR=/data/results/euroc/vins_fusion_stereo-autocal
ROSBAGS_DIR=/data/euroc_mav/rosbags
EST_TOPIC=/vins_fusion/odometry

# Run algorithms on EuRoC dataset
prep_result_folders $RESULTS_DIR
batch_run_vio $LAUNCH_FILE $CONFIG_FILE $ROSBAGS_DIR $RESULTS_DIR $EST_TOPIC
