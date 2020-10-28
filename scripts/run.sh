#!/bin/bash
set -e

# make
source devel/setup.bash
# roslaunch bench benchmark_euroc-vins_mono.launch
# roslaunch bench benchmark_euroc-vins_fusion.launch
# roslaunch bench benchmark_euroc-msckf_vio.launch

# RESULTS_PATH=results/euroc/vins_mono/MH_01_autocal_calib
# RESULTS_PATH=results/euroc/vins_mono/MH_01_euroc_calib
# EST_TOPIC=/vins_fusion/odometry

# RESULTS_PATH=results/euroc/msckf_vio/V1_01_euroc_calib
RESULTS_PATH=results/euroc/msckf_vio/V1_01_autocal_calib
EST_TOPIC=/firefly_sbx/vio/odom

# # Prepare data and run plot script
# python scripts/rosutils/prep_data.py \
#   benchmark_euroc-vins_fusion.bag \
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

# # Compare two different calibrations
# pdfseparate results/euroc/msckf_vio/V1_01_euroc_calib/report.pdf tmp-%04d-1.pdf
# pdfseparate results/euroc/msckf_vio/V1_01_autocal_calib/report.pdf tmp-%04d-2.pdf
# pdfjam tmp-*-*.pdf --nup 1x2 --landscape --outfile report.pdf
# rm -f tmp-*-*.pdf

# cd src/ORB_SLAM3
# chmod +x build.sh
# ./build.sh
