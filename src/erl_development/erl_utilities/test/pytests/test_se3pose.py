""" This test verifies all imports for SE3Pose are correct and verifies some examples
"""
PKG = 'erl_utilities'

import unittest
import numpy as np
import pyErlUtils as utils
from pyErlUtils import SE3Pose


## A sample python unit test
class TestSE3(unittest.TestCase):
    """ This class tests SE3Pose objects and some examples."""

    def setUp(self) -> None:
        pass

    def test_construction(self):
        # Create Poses from different initializations.
        init = np.array([1, 2, 3, .1, .2, .3])  # (X,Y,Z,Ro,Pi,Yaw)
        pose = SE3Pose(init)
        self.assertAlmostEqual(pose.position[0], init[0])
        self.assertAlmostEqual(pose.position[1], init[1])
        self.assertAlmostEqual(pose.position[2], init[2])
        self.assertAlmostEqual(pose.getRoll(), init[3])
        self.assertAlmostEqual(pose.getPitch(), init[4])
        self.assertAlmostEqual(pose.getYaw(), init[5])


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'erl_utilities', TestSE3)
