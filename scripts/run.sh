set -e

# make
source devel/setup.bash
# roslaunch bench benchmark_euroc-vins_mono.launch
roslaunch bench benchmark_euroc-vins_fusion.launch

# python scripts/rosutils/benchmark_vio.py \
#   src/bench/benchmark_euroc-vins_fusion.bag \
#   /vins_estimator/odometry \

# rosrun rpg_trajectory_evaluation \
#   analyze_trajectory_single.py \
#   results/vins_mono/MH_01_easy

# cd src/ORB_SLAM3
# chmod +x build.sh
# ./build.sh
