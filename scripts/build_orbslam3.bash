#!/bin/bash
set -e
cd src/ORB_SLAM3

echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

echo "Uncompress vocabulary ..."
cd ../../../
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
