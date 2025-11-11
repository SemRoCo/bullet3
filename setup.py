from skbuild import setup

setup(
    name='giskardpy_bullet_bindings',
    version='1.0.1',
    description='Custom bullet wrapper used by Giskardpy. The original code was developed by Erwin Coumans, Yunfei Bai, Jasmine Hsu on https://github.com/bulletphysics/bullet3. '
    'Adrian Röfer added a different python wrapper with a more convenient interface for Giskardpy and Simon Stelter adapted it further.',
    url='https://github.com/SemRoCo/bullet3',
    author='Simon Stelter',
    license='zlib',
    platforms='any',
    packages=["giskardpy_bullet_bindings"],
    package_dir={"": ""},
    cmake_install_dir="giskardpy_bullet_bindings",
    cmake_args=["-G", "Unix Makefiles",
                "-DBUILD_PYBULLET=ON",
                "-DBUILD_PYBULLET_NUMPY=ON",
                "-DBT_ENABLE_VHACD=TRUE",
                "-DUSE_DOUBLE_PRECISION=ON",
                "-DBT_USE_EGL=ON",
                "-DCMAKE_BUILD_TYPE=Release"],
    include_package_data=False,
)
