#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR"/config.bash

export DEBIAN_FRONTEND="noninteractive"
export DOWNLOAD_PATH=$DOWNLOAD_PATH
mkdir -p "$PREFIX"
mkdir -p "$DOWNLOAD_PATH"

install() {
  echo "Installing $1 ..." && "$BASEDIR"/install_"$1".bash > /dev/null
}

install_base() {
  apt_install dialog apt-utils git mercurial cmake g++ clang
}

apt_update
install_base
install ros
install ros_pkgs
install boost
install ceres
install eigen
install opencv3
install opengl
install glew
install yamlcpp
install python_pkgs
