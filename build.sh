#!/bin/bash

set -x

git submodule update --init --recursive

export CMAKE_PREFIX_PATH=$(pwd)/devel

mkdir -p octomap/build
cd octomap/build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$CMAKE_PREFIX_PATH
make install -j
cd ../..

mkdir -p build
cd build
cmake ..
make -j

set +x
