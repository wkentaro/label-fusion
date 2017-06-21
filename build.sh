#!/bin/bash

set -x

THIS_DIR=$(pwd)

git submodule update --init --recursive

export CMAKE_PREFIX_PATH=${THIS_DIR}/devel

for ext in octomap/octomap pcl; do
  mkdir -p ${THIS_DIR}/${ext}/build
  cd ${THIS_DIR}/${ext}/build
  cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$CMAKE_PREFIX_PATH
  make install -j
  cd ${THIS_DIR}
done

mkdir -p build
cd build
cmake ..
make -j

set +x
