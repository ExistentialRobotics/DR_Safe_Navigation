""" This test verifies all imports for the Trajectory Utilities are correct,
and Polynomials and Piecewise Polynomials can be constructed correctly."""

PKG = 'erl_utilities'

import unittest
import numpy as np
import pyErlUtils as utils
from pyErlUtils import Polynomial
from pyErlUtils import PiecewisePolynomial


## A sample python unit test
class TestTrajectory(unittest.TestCase):
    """ This class tests creation of Polynomials and Piecewise Polynomials."""

    def create_polynomial(self):
        # Create Simple Polynomials
        t = Polynomial("t", 1)
        x1 = t ** 2
        x2 = -1 * t ** 3 + t ** 2
        # Create Piecewise Polynomial
        breaks = [0, 1, 1.5]
        x_t = PiecewisePolynomial([x1, x2], breaks)


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, 'erl_utilities', TestTrajectory)
