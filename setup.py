#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lisa_profiler'],
    package_dir={'': 'src'},
    install_requires=['sounddevice', 'soundfile', 'pandas']
)

setup(**d)
