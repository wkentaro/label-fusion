#!/bin/bash

set -x

THIS_DIR=$(pwd)

git submodule update --init --recursive

export CMAKE_PREFIX_PATH=${THIS_DIR}/devel

mkdir -p ${THIS_DIR}/octomap/octomap/build
cd ${THIS_DIR}/octomap/octomap/build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$CMAKE_PREFIX_PATH
make install -j
cd ${THIS_DIR}

mkdir -p build
cd build
cmake ..
make -j

set +x
