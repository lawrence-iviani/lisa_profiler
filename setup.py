#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lisa_profiler'],
    package_dir={'': 'src'},
    scripts=['scripts/bag_plot', 'scripts/bag_csv', 'scripts/bag_print'],
    install_requires=['sounddevice', 'soundfile', 'rosbag_pandas', 'pandas']
)

setup(**d)
