#!/bin/sh

cd ../

if [ ! -d "pybind11" ]; then
    git clone https://github.com/pybind/pybind11.git
    cd pybind11
    mkdir build
    cd build
    if [ -z ${ROS_DISTRO} ] || [ ${ROS_DISTRO} = "noetic" ]; then
        cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=3 -DPYBIND11_TEST=OFF
    else
        cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=2 -DPYBIND11_TEST=OFF
    fi
    sudo make install
    cd ../..
fi

cd bullet3

if [ -z ${ROS_DISTRO} ] || [ ${ROS_DISTRO} = "noetic" ]; then
    ./build_cmake_pybullet_3.8_double.sh Release
else
    ./build_cmake_pybullet_2.7_double.sh Release
fi

echo 'export PYTHONPATH=${PYTHONPATH}':"${PWD}/build_cmake/better_python:${PWD}/examples/pybullet" >> ~/.bashrc
