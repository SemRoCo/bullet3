#!/bin/sh

cd ../

if [ ! -d "pybind11" ]; then
    git clone https://github.com/pybind/pybind11.git
    cd pybind11
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=3 -DPYBIND11_TEST=OFF
    sudo make install
    cd ../..
fi

cd bullet3

if [ -z ${ROS_DISTRO} ] || [ ${ROS_DISTRO} = "jazzy" ]; then
    ./build_cmake_pybullet_3.12_double.sh Release
else
    echo 'only ros jazzy is supported'
fi

echo 'export PYTHONPATH=${PYTHONPATH}':"${PWD}/build_cmake/better_python:${PWD}/examples/pybullet" >> ~/.bashrc
