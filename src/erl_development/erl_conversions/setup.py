'''This marks the erl_conversions package to be registered as a Python moduel for importing in other places.'''

#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['py_msg_conversions'],
    package_dir={'': 'src'}
)

setup(**setup_args)