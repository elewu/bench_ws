#!/bin/bash
set -e

# make
source devel/setup.bash

# roslaunch bench benchmark_euroc-okvis.launch

# python scripts/compute_stereo_rectify.py > src/bench/configs/autocal/orbslam3/euroc-stereo_imu.yaml
# ./scripts/bench_euroc-orbslam3.bash

# RESULTS_DIR=/data/results/euroc/orbslam3-autocal
# for DIR in $(ls $RESULTS_DIR); do
# python src/ORB_SLAM3/evaluation/evaluate_ate_scale.py \
#   /data/euroc_mav/$DIR/mav0/state_groundtruth_estimate0/data.csv \
#   $RESULTS_DIR/$DIR/orbslam3_estimate.txt \
#   --verbose >> autocal_results.txt
# done

./scripts/bench_euroc-vins_fusion.bash

# RESULTS_DIR=/data/results/euroc/orbslam3
# for DIR in $(ls $RESULTS_DIR); do
# python src/ORB_SLAM3/evaluation/evaluate_ate_scale.py \
#   /data/euroc_mav/$DIR/mav0/state_groundtruth_estimate0/data.csv \
#   $RESULTS_DIR/$DIR/orbslam3_estimate.txt \
#   --verbose >> euroc_results.txt
# done

# # Compare two different runs
# compare_runs \
#   results/euroc/msckf_vio/V1_01/report.pdf \
#   results/euroc/vins_fusion/V1_01/report.pdf \
#   report.pdf
