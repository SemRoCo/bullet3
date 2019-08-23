import os
import re
import sys
import platform
import subprocess

from distutils.core import setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable,
                      '-DBUILD_PYBULLET=ON', 
                      '-DBUILD_PYBULLET_NUMPY=OFF',
                      '-DUSE_DOUBLE_PRECISION=ON', 
                      '-DCMAKE_BUILD_TYPE=Release', 
                      '-DPYTHON_INCLUDE_DIR=/usr/include/python2.7',
                      '-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7.so']

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)


bpb_sources = sum([['{}/{}'.format(r, f) for f in files if f[-4:] == '.cpp'] for r, _, files in os.walk('betterpybullet')])

# betterpybullet_module = Extension(
#     'betterpybullet',
#     sources=bpb_sources,
#     language='C++',
#     )


# setup(
#     name='betterpybullet',
#     version='0.1.0',
#     description='A better python bullet wrapper using pybind11',
#     ext_modules=[CMakeExtension('betterpybullet', [])],
#     cmdclass=dict(build_ext=CMakeBuild),
#     zip_safe=False)

setup(
    name='betterpybullet',
    version='0.0.1',
    author='Me',
    author_email='rcv@stackoverflow.com',
    description='A better python bullet wrapper using pybind11',
    long_description='',
    ext_modules=[CMakeExtension('betterpybullet', 'better_python')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
    install_requires=[
        'cmake',
    ]
)