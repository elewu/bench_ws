#!/bin/bash
set -e
SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
source $SCRIPT_PATH/benchmark.bash
source $SCRIPT_PATH/../devel/setup.bash
trap 'kill -9 %1' 2


# ORBSLAM3
RESULTS_DIR=/data/results/euroc/
PLATFORM=laptop
ALGO=orbslam3-autocal
CONFIG_FILE=$SCRIPT_PATH/../src/bench/configs/autocal/orbslam3/euroc-stereo_imu.yaml
ROSBAGS_DIR=/data/euroc_mav/rosbags
LAUNCH_FILE=benchmark_euroc-orbslam3-stereo_imu.launch

run_orbslam3() {
  LAUNCH_FILE=$1;
  CONFIG_FILE=$2;
  ROSBAG_INPUT=$3;
  RESULTS_FILE=$4;
  RESULTS_PATH=$(dirname $RESULTS_FILE);

  # Run ORB_SLAM3
  roslaunch bench $LAUNCH_FILE \
    config_file:=$CONFIG_FILE \
    rosbag_input_path:=$ROSBAG_INPUT \
    save_path:=$RESULTS_FILE

  # Convert data from orbslam3 format to rpg_trajectory_evaluation format
  python $SCRIPT_PATH/orbslam3_prep_data.py \
    $RESULTS_PATH/orbslam3_estimate.txt \
    $RESULTS_PATH/stamped_groundtruth.txt \
    $RESULTS_PATH/stamped_traj_estimate.txt

  # # Evaluate results
  # rosrun rpg_trajectory_evaluation \
  #   analyze_trajectory_single.py \
  #   $RESULTS_PATH
  #
  # # Combine plots into a single pdf
  # pdfunite \
  #   $RESULTS_PATH/plots/traj_est/rel_translation_error.pdf \
  #   $RESULTS_PATH/plots/traj_est/rel_translation_error_perc.pdf \
  #   $RESULTS_PATH/plots/traj_est/rel_yaw_error.pdf \
  #   $RESULTS_PATH/plots/traj_est/rotation_error_posyaw_-1.pdf \
  #   $RESULTS_PATH/plots/traj_est/scale_error_posyaw_-1.pdf \
  #   $RESULTS_PATH/plots/traj_est/trajectory_side_posyaw_-1.pdf \
  #   $RESULTS_PATH/plots/traj_est/trajectory_top_posyaw_-1.pdf \
  #   $RESULTS_PATH/plots/traj_est/translation_error_posyaw_-1.pdf \
  #   $RESULTS_PATH/report.pdf
}

batch_run_orbslam3() {
  LAUNCH_FILE=$1;
  CONFIG_FILE=$2;
  ROSBAGS_DIR=$3;

  RESULTS_DIR=$4;
  PLATFORM=$5;

  for ROSBAG in $(ls $ROSBAGS_DIR); do
    ROSBAG_INPUT="$ROSBAGS_DIR/$ROSBAG";
    DS=${ROSBAG/.bag/};
    RESULTS_PATH=$RESULTS_DIR/${PLATFORM}/${ALGO}/${PLATFORM}_${ALGO}_${DS};
    SAVE_PATH="${RESULTS_PATH}/orbslam3_estimate.txt";
    run_orbslam3 $LAUNCH_FILE $CONFIG_FILE $ROSBAG_INPUT $SAVE_PATH
  done
}

batch_analyze_orbslam3_results() {
  RESULTS_DIR=$1;

  for DIR in $(dir $RESULTS_DIR); do
    RESULTS_PATH=$RESULTS_DIR/$DIR;
    analyze_orbslam3_results $RESULTS_PATH;
  done
}

# Run algorithms on EuRoC dataset
prep_result_folders $RESULTS_DIR $PLATFORM $ALGO
batch_run_orbslam3 $LAUNCH_FILE $CONFIG_FILE $ROSBAGS_DIR $RESULTS_DIR $PLATFORM
