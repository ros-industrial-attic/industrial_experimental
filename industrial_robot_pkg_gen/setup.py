#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['industrial_robot_pkg_gen'],
    scripts=['src/industrial_robot_pkg_gen'],
    package_dir={'': 'src'}
)

setup(**d)

