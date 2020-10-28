FROM ros:melodic-perception
ENV CATKIN_WS=/root/catkin_ws
ARG DEBIAN_FRONTEND=noninteractive

# Update Ubuntu Software repository
RUN apt update

# Instal Dependencies
RUN apt-get install -y \
   apt-utils \
   vim \
   vifm \
   tmux \
   cmake \
   libatlas-base-dev \
   libeigen3-dev \
   libgoogle-glog-dev \
   libsuitesparse-dev \
   libceres-dev \
   python-catkin-tools \
   ros-${ROS_DISTRO}-cv-bridge \
   ros-${ROS_DISTRO}-image-transport \
   ros-${ROS_DISTRO}-message-filters \
   ros-${ROS_DISTRO}-tf \
   ros-${ROS_DISTRO}-rosbag

# Setup catkin workspace
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS
RUN catkin init

# Clone VINS-Mono
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git

# Checkout specific release
# WORKDIR $CATKIN_WS/src/VINS-Mono

# Build VINS-Mono
WORKDIR $CATKIN_WS
RUN catkin config --extend /opt/ros/$ROS_DISTRO
RUN catkin build -j2 -DCMAKE_BUILD_TYPE=Release

# # Copy over configurations
# COPY ./configs /root/configs
# RUN rm -rf /root/.tmux
# RUN rm -rf /root/.vim
# RUN rm -rf /root/.vifm
# RUN ln -s /root/configs/tmux /root/.tmux
# RUN ln -s /root/configs/tmux/tmux.conf /root/.tmux
# RUN ln -s /root/configs/vifm /root/.vifm
# RUN ln -s /root/configs/vifm/vifmrc /root/.vifmrc
# RUN ln -s /root/configs/vim /root/.vim
# RUN ln -s /root/configs/vim/vimrc /root/.vimrc

# Setup entry
WORKDIR $CATKIN_WS/src/VINS-Mono
