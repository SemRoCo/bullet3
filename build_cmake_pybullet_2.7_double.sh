#!/bin/sh

if [ "$#" -ge 1 ]; then
    BUILD_TYPE=$1
else
    BUILD_TYPE="Release"    
fi

echo "Building in ${BUILD_TYPE} mode..."

if [ -e CMakeCache.txt ]; then
  rm CMakeCache.txt
fi
mkdir -p build_cmake
cd build_cmake
cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=ON -DUSE_DOUBLE_PRECISION=ON -DBT_USE_EGL=ON -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DPYTHON_EXECUTABLE=/usr/bin/python -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7.so .. || exit 1
make -j $(command nproc 2>/dev/null || echo 12) || exit 1
cd examples
cd pybullet
if [ -e pybullet.dylib ]; then
  ln -f -s pybullet.dylib pybullet.so
fi
if [ -e pybullet_envs ]; then
  rm pybullet_envs
fi
if [ -e pybullet_data ]; then
  rm pybullet_data
fi
if [ -e pybullet_utils ]; then
  rm pybullet_utils
fi
ln -s ../../../examples/pybullet/gym/pybullet_envs .
ln -s ../../../examples/pybullet/gym/pybullet_data .
ln -s ../../../examples/pybullet/gym/pybullet_utils .
echo "Completed build of Bullet."
