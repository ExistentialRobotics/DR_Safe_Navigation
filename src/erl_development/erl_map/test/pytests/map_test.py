"""This script tests importing the ERL Map Python bindings and basic construction of Maps."""
PKG='erl_map'

import unittest
from pyErlMap import GridMap

import numpy as np

## A sample python unit test
class TestErlMap(unittest.TestCase):

    def test_map_create(self):
        """Create a Map from min, max, res data."""
        min = [-10, -10]
        max = [10, 10]
        res = [.25, .25]
        map = GridMap(min, max, res)

        # Check Min
        self.assertLess(np.abs(map.min()[0] + 10.125), res[0])
        self.assertLess(np.abs(map.min()[1] + 10.125), res[0])
        # Check Max
        self.assertLess(np.abs(map.max()[1] - 10.125), res[0])
        self.assertLess(np.abs(map.max()[1] - 10.125), res[0])

        # Check Res
        self.assertEqual(map.res()[0], .25)
        self.assertEqual(map.res()[1], .25)

    def test_map_from_file(self):
        """Create a Map by loading from a data file."""
        map = GridMap()
        map.load_legacy('data/map_2d_col.yaml')

        # Check Min
        self.assertEqual(map.min()[0], -2)
        self.assertEqual(map.min()[1], -2)
        # Check Max
        self.assertEqual(map.max()[0], 1)
        self.assertEqual(map.max()[1], 1)

        # Check Res
        self.assertEqual(map.res()[0], 1)
        self.assertEqual(map.res()[1], 1)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'erl_map', TestErlMap)
