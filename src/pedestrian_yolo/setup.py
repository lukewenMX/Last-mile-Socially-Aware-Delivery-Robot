#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pedestrian_yolo'],
    package_dir={'': 'src'},
    scripts=['bin/pedestrian_yolo_node']
)

setup(**d)
