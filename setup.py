#!/usr/bin/env python3

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

d = generate_distutils_setup(
    packages=['difftrack_mpc'],
    package_dir={'': 'src'}
)

setup(**d)
