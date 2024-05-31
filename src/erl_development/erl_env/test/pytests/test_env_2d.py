PKG='erl_env'

import unittest
from pyErlMap import GridMap
import pyErlEnv as env

import numpy as np


## A sample python unit test
class TestErlEnv2D(unittest.TestCase):

    def setUp(self):
        """ Initialize a 2D Environment for Testing"""
        min = [-10, -10]
        max = [10, 10]
        res = [1, 1]
        map = GridMap(min, max, res)
        self.env = env.Environment2D(map)

    def test_env_2d(self):
        """ Test the GetSuccessors Function"""
        # Set Start coordinate for Plotting some Primitives
        start = [0.0, 0.0]  # (X, Y)
        # Get Successors from Environment.
        succ_list = self.env.getSuccessors(start)
        self.assertEqual(len(succ_list), 8)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'erl_env', TestErlEnv2D)
