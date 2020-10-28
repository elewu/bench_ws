#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

apt install cmake
apt_install libgtest-dev

cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo ln -fs *.a /usr/lib
