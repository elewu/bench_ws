#!/bin/bash
set -e

# make
source devel/setup.bash
source scripts/benchmark.bash

# python scripts/compute_stereo_rectify.py \
#    --calib_file src/bench/configs/kalibr/camchain.yaml \
#    --calib_format kalibr \
#    --mode VO \
#    > src/bench/configs/kalibr/euroc-stereo.yaml

# python scripts/compute_stereo_rectify.py \
#    --calib_file /tmp/calib_stereo.yaml \
#    --calib_format autocal \
#    --mode VO \
#    > src/bench/configs/autocal/orbslam3/euroc-stereo.yaml

# ./scripts/benchmark_euroc-orbslam3.bash

# RESULTS_DIR=/data/results/euroc
# ANALYSIS_DIR=/data/results/euroc/analysis_vo
# ANALYSIS_YAML=/data/results/euroc/euroc_vo_analysis.yaml
# analyze_euroc_runs $RESULTS_DIR $ANALYSIS_DIR $ANALYSIS_YAML

# rosrun \
#   rpg_trajectory_evaluation \
#   analyze_trajectory_single.py \
#   /tmp/laptop_orbslam3-autocal_MH_01
