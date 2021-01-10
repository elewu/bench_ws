#!/bin/bash
set -e
SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

prep_result_folders() {
  # Prepare results folders
  RESULTS_DIR=$1;
  PLATFORM=$2;
  ALGO=$3;
  DATASETS=(MH_01 MH_02 MH_03 MH_04 MH_05
            V1_01 V1_02 V1_03
            V2_01 V2_02 V2_03);

  # Unpack groundtruth data
  DATA_PATH=$SCRIPT_PATH/../data
  tar -xf $DATA_PATH/euroc.tar.bz2 -C $DATA_PATH

  for DS in ${DATASETS[@]}; do
    RESULTS_PATH=$RESULTS_DIR/${PLATFORM}/${ALGO}/${PLATFORM}_${ALGO}_${DS};
    mkdir -p $RESULTS_PATH;

    cd $RESULTS_PATH;
    ln -fs $DATA_PATH/eval_cfg.yaml .;
    ln -fs $DATA_PATH/${DS}_groundtruth.txt stamped_groundtruth.txt;
    cd ~-;
  done
}

run_vio() {
  LAUNCH_FILE=$1;
  CONFIG_FILE=$2;
  ROSBAG_INPUT=$3;
  ROSBAG_OUTPUT=$4;
  EST_TOPIC=$5;

  # Run VIO
  roslaunch bench $LAUNCH_FILE \
    config_file:=$CONFIG_FILE \
    rosbag_input_path:=$ROSBAG_INPUT \
    rosbag_output_path:=$ROSBAG_OUTPUT

  # # Prepare data and run plot script
  # RESULTS_PATH=$(dirname $ROSBAG_OUTPUT);
  # EST_TOPIC=$EST_TOPIC;
  #
  # python scripts/rosutils/prep_data.py \
  #   $RESULTS_PATH/estimation.bag \
  #   $EST_TOPIC \
  #   $RESULTS_PATH/stamped_traj_estimate.txt
  #
  # rosrun rpg_trajectory_evaluation \
  #   analyze_trajectory_single.py \
  #   $RESULTS_PATH
  #
  # # Put plots together in a single pdf
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

batch_run_vio() {
  LAUNCH_FILE=$1;
  CONFIG_FILE=$2;
  ROSBAGS_DIR=$3;
  RESULTS_DIR=$4;
  EST_TOPIC=$5;

  for ROSBAG in $(ls $ROSBAGS_DIR); do
    ROSBAG_INPUT="$ROSBAGS_DIR/$ROSBAG";
    ROSBAG_OUTPUT="$RESULTS_DIR/${ROSBAG/.bag/}_estimation.bag";
    run_vio $LAUNCH_FILE $CONFIG_FILE $ROSBAG_INPUT $ROSBAG_OUTPUT $EST_TOPIC
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

analyze_euroc_runs() {
  RESULTS_DIR=$1;
  ANALYSIS_DIR=$2;
  ANALYSIS_YAML=$3;

  rosrun \
    rpg_trajectory_evaluation \
    analyze_trajectories.py \
    $ANALYSIS_YAML \
    --output_dir=$ANALYSIS_DIR \
    --results_dir=$RESULTS_DIR \
    --platform laptop \
    --odometry_error_per_dataset \
    --plot_trajectories \
    --rmse_table \
    --rmse_boxplot

  pdfunite \
    $ANALYSIS_DIR/laptop_MH_01_results/MH_01_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_MH_02_results/MH_02_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_MH_03_results/MH_03_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_MH_04_results/MH_04_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_MH_05_results/MH_05_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_V1_01_results/V1_01_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_V1_02_results/V1_02_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_V1_03_results/V1_03_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_V2_01_results/V2_01_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_V2_02_results/V2_02_trans_rot_error.pdf \
    $ANALYSIS_DIR/laptop_V2_03_results/V2_03_trans_rot_error.pdf \
    $ANALYSIS_DIR/report.pdf
}

