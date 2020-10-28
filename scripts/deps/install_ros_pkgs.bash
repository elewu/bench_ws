set -e
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

apt_install ros-${ROS_DISTRO}-random-numbers
apt_install ros-${ROS_DISTRO}-pcl-conversions
apt_install ros-${ROS_DISTRO}-pcl-msgs
apt_install ros-${ROS_DISTRO}-pcl-ros
apt_install ros-${ROS_DISTRO}-cv-bridge
apt_install ros-${ROS_DISTRO}-image-transport
apt_install ros-${ROS_DISTRO}-message-filters
apt_install ros-${ROS_DISTRO}-tf
apt_install ros-${ROS_DISTRO}-rosbag
