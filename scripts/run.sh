#!/bin/bash
set -e

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

  roslaunch bench $LAUNCH_FILE \
    rosbag_input_path:=$ROSBAG_INPUT \
    rosbag_output_path:=$ROSBAG_OUTPUT
}

run_orbslam3() {
  LAUNCH_FILE=$1;
  ROSBAG_INPUT=$2;
  SAVE_OUTPUT=$3;

  roslaunch bench $LAUNCH_FILE \
    rosbag_input_path:=$ROSBAG_INPUT \
    save_path:=$SAVE_OUTPUT
}

analyze_results() {
  RESULTS_PATH=$1;
  EST_TOPIC=$2;

  # Prepare data and run plot script
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

analyze_orbslam3_results() {
  RESULTS_PATH=$1;

  rosrun rpg_trajectory_evaluation \
    analyze_trajectory_single.py \
    $RESULTS_PATH

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

  for ROSBAG in $(ls $ROSBAGS_DIR); do
    ROSBAG_INPUT="$ROSBAGS_DIR/$ROSBAG";
    ROSBAG_OUTPUT="$RESULTS_DIR/${ROSBAG/.bag/}/estimation.bag";
    run_vio $LAUNCH_FILE $ROSBAG_INPUT $ROSBAG_OUTPUT
  done
}

batch_run_orbslam3() {
  LAUNCH_FILE=$1;
  ROSBAGS_DIR=$2;
  RESULTS_DIR=$3;

  for ROSBAG in $(ls $ROSBAGS_DIR); do
    ROSBAG_INPUT="$ROSBAGS_DIR/$ROSBAG";
    SAVE_PATH="$RESULTS_DIR/${ROSBAG/.bag/}/stamped_traj_estimate.txt";
    run_orbslam3 $LAUNCH_FILE $ROSBAG_INPUT $SAVE_PATH
  done
}

batch_analyze_results() {
  RESULTS_DIR=$1;
  EST_TOPIC=$2;

  for DIR in $(dir $RESULTS_DIR); do
    RESULTS_PATH=$RESULTS_DIR/$DIR;
    analyze_results $RESULTS_PATH $EST_TOPIC;
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

# python scripts/compute_stereo_rectify.py > rectify.txt

# make
source devel/setup.bash
# ROSBAGS_DIR=/data/euroc_mav/rosbags

# roslaunch bench benchmark_euroc-orbslam3-stereo_imu.launch
# python src/ORB_SLAM3/evaluation/evaluate_ate_scale.py \
#   /data/euroc_mav/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv \
#   ./estimate-euroc.txt \
#   --plot orbslam3_V1_01-euroc.pdf \
#   --verbose
# python src/ORB_SLAM3/evaluation/evaluate_ate_scale.py \
#   /data/euroc_mav/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv \
#   ./estimate-autocal.txt \
#   --plot orbslam3_V1_01-autocal.pdf \
#   --verbose
# python src/ORB_SLAM3/evaluation/evaluate_ate_scale.py \
#   /data/euroc_mav/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv \
#   ./estimate-kalibr.txt \
#   --plot orbslam3_V1_01-kalibr.pdf \
#   --verbose

# # VINS-Fusion
# RESULTS_DIR=$PWD/results/euroc/vins_fusion
# LAUNCH_FILE=benchmark_euroc-vins_fusion.launch
# EST_TOPIC=/vins_fusion/odometry

# # VINS-Mono
# RESULTS_DIR=$PWD/results/euroc/vins_mono
# LAUNCH_FILE=benchmark_euroc-vins_mono.launch
# EST_TOPIC=/vins_estimator/odometry

# # Stereo-MSCKF
# RESULTS_DIR=$PWD/results/euroc/msckf_vio
# LAUNCH_FILE=benchmark_euroc-msckf_vio.launch
# EST_TOPIC=/firefly_sbx/vio/odom

# ORBSLAM3
# RESULTS_DIR=$PWD/results/euroc/orbslam3
# LAUNCH_FILE=benchmark_euroc-orbslam3-stereo_imu.launch
# prep_result_folders $RESULTS_DIR

# run_orbslam3 \
#   $LAUNCH_FILE \
#   /data/euroc_mav/rosbags/MH_01.bag \
#   $PWD/results/euroc/orbslam3/MH_01/estimate2.txt
# python src/ORB_SLAM3/evaluation/evaluate_ate_scale.py \
#   /data/euroc_mav/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
#   ./results/euroc/orbslam3/MH_01/estimate2.txt \
#   --plot orbslam3_MH_01.pdf \
#   --save ./results/euroc/orbslam3/MH_01/estimate_aligned2.txt \
#   --verbose

# analyze_orbslam3_results $PWD/results/euroc/orbslam3/MH_01
# batch_run_orbslam3 $LAUNCH_FILE $ROSBAGS_DIR $RESULTS_DIR


# # Run algorithms on EuRoC dataset
# prep_result_folders $RESULTS_DIR
# batch_run_vio $LAUNCH_FILE $ROSBAGS_DIR $RESULTS_DIR
# batch_analyze_results $RESULTS_DIR $EST_TOPIC

# # Compare two different runs
# compare_runs \
#   results/euroc/msckf_vio/V1_01/report.pdf \
#   results/euroc/vins_fusion/V1_01/report.pdf \
#   report.pdf

# cd src/ORB_SLAM3
# chmod +x build.sh
# ./build.sh
