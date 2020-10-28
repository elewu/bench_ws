# make
source devel/setup.bash
# roslaunch bench benchmark_euroc-vins_mono.launch

# python scripts/rosutils/benchmark_vio.py \
#   src/bench/benchmark_euroc-vins_mono.bag \
#   /vins_estimator/odometry \
#   /leica/position

rosrun rpg_trajectory_evaluation analyze_trajectory_single.py results

# cd src/ORB_SLAM3
# chmod +x build.sh
# ./build.sh
