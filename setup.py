from skbuild import setup

setup(
    name='betterpybullet',
    version='0.1.0',
    long_description='',
    url='https://github.com/SemRoCo/bullet3',
    author='',
    author_email='',
    platforms='any',
    packages=["betterpybullet"],
    package_dir={"": ""},
    cmake_install_dir="betterpybullet",
    cmake_args=["-G", "Unix Makefiles",
                "-DBUILD_PYBULLET=ON",
                "-DBUILD_PYBULLET_NUMPY=ON",
                "-DBT_ENABLE_VHACD=TRUE",
                "-DUSE_DOUBLE_PRECISION=ON",
                "-DBT_USE_EGL=ON",
                "-DCMAKE_BUILD_TYPE=Release"],
    include_package_data=False,
)
