'''This marks the py_visualization package to be registered as a Python module for importing in other places.'''

#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['py_visualization'],
    package_dir={'': 'src'}
)

setup(**setup_args)