#!/bin/bash
set -e
cd src/ORB_SLAM3

echo "Configuring and building ORB_SLAM3 ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
