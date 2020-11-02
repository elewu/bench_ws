bench_ws
========

<a href="https://github.com/chutsu/bench_ws/actions?query=ci">
  <img src="https://github.com/chutsu/bench_ws/workflows/ci/badge.svg">
</a>

A catkin workspace to benchmark different state estimation algorithms, namely:

- [VINS-Mono][VINS-Mono]
- [VINS-Fusion][VINS-Fusion]
- [ORBSLAM3][ORBSLAM3]
- [Stereo-MSCKF][Stereo-MSCKF]
- [OKVIS][OKVIS]


Build
-----

The build process is fairly automated, this repo assumes you are running on a
linux machine with Ubuntu 18.04 installed. Then to use this repo you would
issue the following commands,

    make deps
    make submodules
    make build

which installs dependencies, pull the git submoduels and builds this workspace.


Run on EuRoC dataset
--------------------

First obtain the [EuRoC][EuRoC] dataset. The launch files provided by this
project by default assumes you have a directory called `/data/euroc_mav/rosbags`
and that it contains the rosbags as follows:

    MH_01.bag
    MH_02.bag
    MH_03.bag
    MH_04.bag
    MH_05.bag
    V1_01.bag
    V1_02.bag
    V1_03.bag
    V2_01.bag
    V2_02.bag
    V2_03.bag

Note how the `easy`, `medium` and `difficult` suffix tags are removed.

To run any of the supported state-estimation algorithm on a EuRoC dataset use
one of the following roslaunch files in the `bench` package.

    benchmark_euroc-msckf_vio.launch
    benchmark_euroc-orbslam3-mono.launch
    benchmark_euroc-orbslam3-stereo.launch
    benchmark_euroc-orbslam3-stereo_imu.launch
    benchmark_euroc-vins_fusion.launch
    benchmark_euroc-vins_mono.launch

Most of these launch files have the following launch argments:

    rosbag_input_path: Path to ROS bag to run the algorithm against
    rosbag_outfile: Filename to save the estimation in a ROS bag
    rosbag_output_path: Full path to where the estimation is saved to
    config_file:: Config / calibration file for the specific algorithm

see individual launch files for default and more options. These roslaunch file
arguments can be over-ridden in the commandline while performing a `roslaunch`
so that you don't have to directly change the launch file yourself, for
example:

    roslanch bench benchmark_euroc-msckf_vio.launch \
      config_file:=<some new config file> \
      rosbag_input_path:=<some new ros bag>


LICENCE
-------

MIT


[VINS-Mono]: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
[VINS-Fusion]:https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
[ORBSLAM3]: https://github.com/UZ-SLAMLab/ORB_SLAM3
[Stereo-MSCKF]: https://github.com/KumarRobotics/msckf_vio
[OKVIS]: https://github.com/ethz-asl/okvis

[EuRoC]: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
