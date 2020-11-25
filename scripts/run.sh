#!/bin/bash
set -e

# make
source devel/setup.bash

# python scripts/compute_stereo_rectify.py > src/bench/configs/kalibr/euroc-stereo_imu.yaml

# ./scripts/benchmark_euroc-vins_fusion.bash
# ./scripts/benchmark_euroc-orbslam3.bash

# rosrun \
#   rpg_trajectory_evaluation \
#   analyze_trajectories.py \
#   /data/results/euroc/euroc_analysis.yaml \
#   --output_dir=/data/results/euroc/analysis \
#   --results_dir=/data/results/euroc \
#   --platform laptop \
#   --odometry_error_per_dataset \
#   --plot_trajectories \
#   --rmse_table \
#   --rmse_boxplot

# rosrun \
#   rpg_trajectory_evaluation \
#   analyze_trajectory_single.py \
#   /tmp/laptop_orbslam3-autocal_MH_01

# pdfunite \
#   /data/results/euroc/analysis/laptop_MH_01_results/MH_01_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_MH_02_results/MH_02_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_MH_03_results/MH_03_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_MH_04_results/MH_04_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_MH_05_results/MH_05_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_V1_01_results/V1_01_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_V1_02_results/V1_02_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_V1_03_results/V1_03_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_V2_01_results/V2_01_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_V2_02_results/V2_02_trans_rot_error.pdf \
#   /data/results/euroc/analysis/laptop_V2_03_results/V2_03_trans_rot_error.pdf \
#   /data/results/euroc/analysis/report.pdf
