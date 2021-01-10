#!/bin/bash
set -e

# make
source devel/setup.bash
source scripts/benchmark.bash

# python scripts/compute_stereo_rectify.py \
#    --calib_file /tmp/calib_results.yaml \
#    --calib_format autocal \
#    --mode VIO \
#    > configs/autocal/orbslam3/euroc-stereo_imu.yaml

# python scripts/compute_stereo_rectify.py \
#    --calib_file src/bench/configs/kalibr/camchain.yaml \
#    --calib_format kalibr \
#    --mode VO \
#    > src/bench/configs/kalibr/euroc-stereo.yaml

# ./scripts/benchmark_euroc-orbslam3-stereo.bash
./scripts/benchmark_euroc-orbslam3-stereo_imu.bash

# RESULTS_DIR=/data/results/euroc
# ANALYSIS_DIR=/data/results/euroc/analysis_vo
# ANALYSIS_YAML=/data/results/euroc/euroc_vo_analysis.yaml
# analyze_euroc_runs $RESULTS_DIR $ANALYSIS_DIR $ANALYSIS_YAML

# rosrun \
#   rpg_trajectory_evaluation \
#   analyze_trajectory_single.py \
#   /tmp/laptop_orbslam3-autocal_MH_01
