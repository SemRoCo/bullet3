from skbuild import setup

setup(
    name='betterpybullet',
    version='0.1.0',
    description=
    '',
    long_description=
    '',
    url='https://github.com/SemRoCo/bullet3',
    author='',
    author_email='',
    license='zlib',
    platforms='any',
    keywords=[
        'physics simulation', 'robotics',
        'collision detection', 'opengl'
    ],
    packages=["better_python"],
    cmake_install_dir="better_python",
    cmake_args=["-DBUILD_PYBULLET=ON",
                "-DBUILD_PYBULLET_NUMPY=ON",
                "-DBT_ENABLE_VHACD=TRUE",
                "-DUSE_DOUBLE_PRECISION=ON",
                "-DBT_USE_EGL=ON",
                "-DCMAKE_BUILD_TYPE=Release"],
    classifiers=[
        'License :: OSI Approved :: zlib/libpng License',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        "Topic :: Games/Entertainment :: Simulation",
        'Framework :: Robot Framework'
    ],
    # python_requires='>=3.8',
    zip_safe=False,
)
