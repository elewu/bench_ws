bench_ws
========

<a href="https://github.com/chutsu/bench_ws/actions?query=ci">
  <img src="https://github.com/chutsu/bench_ws/workflows/ci/badge.svg">
</a>

Just a benchmark catkin workspace to compare against different state estimation
algorithms, namely:

- [VINS-Mono][VINS-Mono]
- [VINS-Fusion][VINS-Fusion]
- [ORBSLAM3][ORBSLAM3]
- [Stereo-MSCKF][Stereo-MSCKF]


Build
-----

The build process is fairly automated, this repo assumes you are running on a
linux machine with Ubuntu 18.04 installed. Then to use this repo you would
issue the following commands,

    make deps
    make submodules
    make build

which installs dependencies, pull the git submoduels and builds this workspace.


Run
---

To run VIO on [EuRoC][EuRoC] dataset, have a look in these roslaunch files:

    src/bench/launch/benchmark_euroc-vins_fusion.launch
    src/bench/launch/benchmark_euroc-vins_mono.launch

Or to run them simply do:

    roslaunch bench benchmark_euroc-vins_fusion.launch
    roslaunch bench benchmark_euroc-vins_mono.launch


LICENCE
-------

MIT


[VINS-Mono]: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
[VINS-Fusion]:https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
[ORBSLAM3]: https://github.com/UZ-SLAMLab/ORB_SLAM3
[Stereo-MSCKF]: https://github.com/KumarRobotics/msckf_vio

[EuRoC]: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
