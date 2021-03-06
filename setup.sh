#!/usr/bin/env bash
# usage: 编译项目

project_path=$(pwd)
git submodule update --recursive --init
cd $project_path
mkdir -p build
cd build
cmake ..
make -j
chmod 777 /dev/ttyUSB0
./WLX