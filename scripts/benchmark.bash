#!/bin/bash
set -e
SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

prep_result_folders() {
  # Prepare results folders
  RESULTS_DIR=$1;
  mkdir -p $RESULTS_DIR;
  mkdir -p $RESULTS_DIR/MH_01;
  mkdir -p $RESULTS_DIR/MH_02;
  mkdir -p $RESULTS_DIR/MH_03;
  mkdir -p $RESULTS_DIR/MH_04;
  mkdir -p $RESULTS_DIR/MH_05;
  mkdir -p $RESULTS_DIR/V1_01;
  mkdir -p $RESULTS_DIR/V1_02;
  mkdir -p $RESULTS_DIR/V1_03;
  mkdir -p $RESULTS_DIR/V2_01;
  mkdir -p $RESULTS_DIR/V2_02;
  mkdir -p $RESULTS_DIR/V2_03;

  # # Unpack groundtruth data
  # GND_PATH=$RESULTS_DIR/ground_truth
  # mkdir -p $GND_PATH
  # tar -xzvf $SCRIPT_PATH/../data/euroc.tar.bz2 -C $GND_PATH

  # Prepare eval_cfg.yaml and groudtruth files
  for DIR in $(dir $RESULTS_DIR); do
    cd $RESULTS_DIR/$DIR;
    ln -fs ../../eval_cfg.yaml .;
    ln -fs `find ../../ -name "${DIR}_groundtruth.txt"` stamped_groundtruth.txt;
    cd ~-;
  done
}

run_vio() {
  LAUNCH_FILE=$1;
  ROSBAG_INPUT=$2;
  ROSBAG_OUTPUT=$3;
  EST_TOPIC=$4;

  # Run VIO
  roslaunch bench $LAUNCH_FILE \
    rosbag_input_path:=$ROSBAG_INPUT \
    rosbag_output_path:=$ROSBAG_OUTPUT

  # Prepare data and run plot script
  RESULTS_PATH=$(dirname $ROSBAG_OUTPUT);
  EST_TOPIC=$EST_TOPIC;

  python scripts/rosutils/prep_data.py \
    $RESULTS_PATH/estimation.bag \
    $EST_TOPIC \
    $RESULTS_PATH/stamped_traj_estimate.txt

  rosrun rpg_trajectory_evaluation \
    analyze_trajectory_single.py \
    $RESULTS_PATH

  # Put plots together in a single pdf
  pdfunite \
    $RESULTS_PATH/plots/traj_est/rel_translation_error.pdf \
    $RESULTS_PATH/plots/traj_est/rel_translation_error_perc.pdf \
    $RESULTS_PATH/plots/traj_est/rel_yaw_error.pdf \
    $RESULTS_PATH/plots/traj_est/rotation_error_posyaw_-1.pdf \
    $RESULTS_PATH/plots/traj_est/scale_error_posyaw_-1.pdf \
    $RESULTS_PATH/plots/traj_est/trajectory_side_posyaw_-1.pdf \
    $RESULTS_PATH/plots/traj_est/trajectory_top_posyaw_-1.pdf \
    $RESULTS_PATH/plots/traj_est/translation_error_posyaw_-1.pdf \
    $RESULTS_PATH/report.pdf
}

batch_run_vio() {
  LAUNCH_FILE=$1;
  ROSBAGS_DIR=$2;
  RESULTS_DIR=$3;
  EST_TOPIC=$4;

  for ROSBAG in $(ls $ROSBAGS_DIR); do
    ROSBAG_INPUT="$ROSBAGS_DIR/$ROSBAG";
    ROSBAG_OUTPUT="$RESULTS_DIR/${ROSBAG/.bag/}/estimation.bag";
    run_vio $LAUNCH_FILE $ROSBAG_INPUT $ROSBAG_OUTPUT $EST_TOPIC
  done
}

compare_runs() {
  RUN1_PDF = $1;
  RUN2_PDF = $2;
  OUTPUT_PATH = $3;

  pdfseparate $RUN1_PDF tmp-%04d-1.pdf
  pdfseparate $RUN2_PDF tmp-%04d-2.pdf
  pdfjam tmp-*-*.pdf --nup 1x2 --landscape --outfile $3
  rm -f tmp-*-*.pdf
}

