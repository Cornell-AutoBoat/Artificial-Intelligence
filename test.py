import unittest
from src.buoys import Buoy
from src.control_tasks import utils
from src import SFR
import numpy as np


class TestSeen(unittest.TestCase):
    def test_one_object_true(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 0.0, 0.0)
        s = set([o1])
        self.assertEqual(utils.seen(o1, s), True, "Could not find buoy in list")

    def test_one_object_false(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0)
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), False, "Wrongly identified buoy")

    def test_simple_mult_object_true(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0)
        o3 = Buoy("blue-buoy", 0.0, 0.0)
        s = set([o1, o2, o3])
        self.assertEqual(utils.seen(o1, s), True, "Could not find buoy in list")

    def test_same_label_diff_coords(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 0.0, 1.0)
        o2 = Buoy("red-buoy", 0.0, 3.0)
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), False, "Wrongly identified buoy based on coordinates")

    def test_same_label_close_coords(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 0.0, 1.0)
        o2 = Buoy("red-buoy", 0.0, 1.2)
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), True, "Couldn't identify that buoys were the same")

    def test_same_label_close_coords_mult_axes(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 1.0, 2.0)
        o2 = Buoy("red-buoy", 1.5, 2.5)
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), True, "Couldn't identify that buoys were the same")

    def test_seen_unaligned_frames(self):
        SFR.heading = 3 * np.pi / 4
        SFR.tx, SFR.ty = 4, 4
        o1 = Buoy("red-buoy", 5*np.sqrt(2)/2, 3*np.sqrt(2)/2)  # local
        o2 = Buoy("red-buoy", 5.0, 8.0)  # global
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), True, "Couldn't identify that buoys were the same")

    def test_not_seen_unaligned_frames(self):
        SFR.heading = 3 * np.pi / 4
        SFR.tx, SFR.ty = 4, 4
        o1 = Buoy("red-buoy", 5.0, 8.0)  # local
        o2 = Buoy("red-buoy", 5.0, 8.0)  # global
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), False, "Couldn't identify that buoys were the same")


class TestMapToGlobal(unittest.TestCase):
    def test_zero_zero_origin_no_heading(self):
        x = 0.0
        y = 0.0
        SFR.tx = 0.0
        SFR.ty = 0.0
        SFR.tz = 0.0
        SFR.heading = 0.0
        self.assertEqual(utils.map_to_global(x, y), [0.0, 0.0],
                         "Wrong global coordinates for local (0.0, 0.0) at origin")

    def test_three_three_origin_no_heading(self):
        x = 3.0
        y = 3.0
        SFR.tx = 0.0
        SFR.ty = 0.0
        SFR.tz = 0.0
        SFR.heading = 0.0
        self.assertEqual(utils.map_to_global(x, y), [3.0, -3.0],
                         "Wrong global coordinates for local (3.0, 3.0) at origin")

    def test_three_three_SFR_no_heading(self):
        x = 3.0
        y = 3.0
        SFR.tx = 3.0
        SFR.ty = 3.0
        SFR.tz = 0.0
        SFR.heading = 0.0
        self.assertEqual(utils.map_to_global(x, y), [6.0, 0.0],
                         "Wrong global coordinates for local (3.0, 3.0) not at origin")

    def test_two_two_SFR_no_heading(self):
        x = 2.0
        y = 2.0
        SFR.tx = 4.0
        SFR.ty = 4.0
        SFR.tz = 0.0
        SFR.heading = 0.0
        self.assertEqual(utils.map_to_global(x, y), [6.0, 2.0],
                         "Wrong global coordinates for local (2.0, 2.0) not at origin")

    def test_one_two_SFR_no_heading(self):
        x = 1.0
        y = 2.0
        SFR.tx = 4.0
        SFR.ty = 2.0
        SFR.tz = 0.0
        SFR.heading = 0.0
        self.assertEqual(utils.map_to_global(x, y), [6.0, 1.0],
                         "Wrong global coordinates for local (1.0, 2.0) not at origin")

    def test_zero_zero_origin_zero_theta(self):
        x = 0.0
        y = 0.0
        SFR.tx = 0.0
        SFR.ty = 0.0
        SFR.tz = 0.0
        SFR.heading = np.pi/2
        self.assertEqual(utils.map_to_global(x, y), [
                         0.0, 0.0], "Wrong global coordinates for local (0.0, 0.0) at origin with zero theta")

    def test_three_three_origin_zero_theta(self):
        x = 3.0
        y = 3.0
        SFR.tx = 0.0
        SFR.ty = 0.0
        SFR.tz = 0.0
        SFR.heading = np.pi/2
        self.assertEqual(utils.map_to_global(x, y), [
                         3.0, 3.0], "Wrong global coordinates for local (3.0, 3.0) at origin with zero theta")

    def test_three_three_SFR_zero_theta(self):
        x = 3.0
        y = 3.0
        SFR.tx = 3.0
        SFR.ty = 3.0
        SFR.tz = 0.0
        SFR.heading = np.pi/2
        self.assertEqual(utils.map_to_global(x, y), [
                         6.0, 6.0], "Wrong global coordinates for local (3.0, 3.0) not at origin with zero theta")

    def test_one_two_SFR_positive_theta(self):
        x = 1.0
        y = 2.0
        SFR.tx = 4.0
        SFR.ty = 2.0
        SFR.tz = 0.0
        SFR.heading = np.pi
        self.assertEqual(utils.map_to_global(x, y), [
                         2.0, 3.0], "Wrong global coordinates for local (1.0, 2.0) not at origin with positive theta")

    def test_zero_zero_origin_negative_theta(self):
        x = 0.0
        y = 0.0
        SFR.tx = 0.0
        SFR.ty = 0.0
        SFR.tz = 0.0
        SFR.heading = (-3 * np.pi)/4
        self.assertEqual(utils.map_to_global(x, y), [
                         0.0, 0.0], "Wrong global coordinates for local (0.0, 0.0) at origin with negative theta")

    def test_two_two_SFR_negative_theta(self):
        x = 2.0
        y = 2.0
        SFR.tx = 4.0
        SFR.ty = 4.0
        SFR.tz = 0.0
        SFR.heading = (-3 * np.pi)/4
        self.assertEqual(utils.map_to_global(x, y), [
                         1.1715728752538102, 4.0], "Wrong global coordinates for local (2.0, 2.0) not at origin with negative theta")

    def test_one_two_SFR_negative_theta(self):
        x = 1.0
        y = 2.0
        SFR.tx = 4.0
        SFR.ty = 2.0
        SFR.tz = 0.0
        SFR.heading = (-3 * np.pi)/4
        self.assertEqual(utils.map_to_global(x, y), [1.8786796564403572, 1.2928932188134525],
                         "Wrong global coordinates for local (1.0, 2.0) not at origin with negative theta")

    def test_neg_five_two_SFR_negative_theta(self):
        x = -5.0
        y = 2.0
        SFR.tx = 4.0
        SFR.ty = -1.0
        SFR.tz = 0.0
        SFR.heading = (-3 * np.pi)/4
        self.assertEqual(utils.map_to_global(x, y), [6.121320343559642, -5.949747468305833],
                         "Wrong global coordinates for local (-5.0, 2.0) not at origin with negative theta")

    def test_five_neg_two_SFR_negative_theta(self):
        x = 5.0
        y = -2.0
        SFR.tx = 4.0
        SFR.ty = -1.0
        SFR.tz = 0.0
        SFR.heading = (-3 * np.pi)/4
        self.assertEqual(utils.map_to_global(x, y), [1.878679656440358, 3.9497474683058327],
                         "Wrong global coordinates for local (5.0, -2.0) not at origin with negative theta")

    def test_neg_five_neg_two_SFR_negative_theta(self):
        x = -5.0
        y = -2.0
        SFR.tx = 4.0
        SFR.ty = -1.0
        SFR.tz = 0.0
        SFR.heading = (-3 * np.pi)/4
        self.assertEqual(utils.map_to_global(x, y), [8.949747468305834, -3.121320343559643],
                         "Wrong global coordinates for local (5.0, -2.0) not at origin with negative theta")

    def test_five_neg_two_neg_SFR_negative_theta(self):
        x = -5.0
        y = -2.0
        SFR.tx = -4.0
        SFR.ty = -1.0
        SFR.tz = 0.0
        SFR.heading = (-3 * np.pi)/4
        self.assertEqual(utils.map_to_global(x, y), [0.9497474683058327, -3.121320343559643],
                         "Wrong global coordinates for local (5.0, -2.0) not at origin with negative theta")


class TestGetMidpoint(unittest.TestCase):
    def test_get_midpoint_both_origin(self):
        SFR.tx = 0
        SFR.ty=0
        o1 = Buoy("test-buoy1", 0.0, 0.0, 0.0)
        o11 = Buoy("test-buoy11", 0.0, 0.0, 0.0)
        mid1 = [0.0, 0.0]
        self.assertEqual(utils.get_midpoint(o1, o11), mid1, "Unequal midpoint")
    
    def test_get_midpoint_one_origin_one_not_origin(self):
        o1 = Buoy("test-buoy1", 0.0, 0.0, 0.0)
        o11 = Buoy("test-buoy11", 1.0, 2.0, 0.0)
        mid1 = [0.5, 1.0]
        self.assertEqual(utils.get_midpoint(o1, o11), mid1, "Unequal midpoint")
    
    def test_get_midpoint_both_not_at_origin(self):
        o1 = Buoy("test-buoy1", 5.0, 2.0, 0.0)
        o11 = Buoy("test-buoy11", 1.0, 2.0, 0.0)
        mid1 = [3.0, 2.0]
        self.assertEqual(utils.get_midpoint(o1, o11), mid1, "Unequal midpoint")

    def test_get_midpoint_two_close_points(self):
        o1 = Buoy("test-buoy1", 5.22, 3.2, 0.0)
        o11 = Buoy("test-buoy11", 5.2, 3.0, 0.0)
        mid1 = [5.21, 3.1]
        self.assertEqual(utils.get_midpoint(o1, o11), mid1, "Unequal midpoint")

    def test_get_midpoint_two_far_points(self):
        o1 = Buoy("test-buoy1", 3.1, 2.1, 0.0)
        o11 = Buoy("test-buoy11", 1000.2, 20000.1, 0.0)
        mid1 = [501.65, 10001.1]
        self.assertEqual(utils.get_midpoint(o1, o11), mid1, "Unequal midpoint")


class TestGetExtendedMidpoint(unittest.TestCase):
    pass


class TestGetShiftedExtendedMidpoint(unittest.TestCase):
    pass


class TestGetExtendedBuoy(unittest.TestCase):
    pass


class TestGetYaw(unittest.TestCase):
    pass


class TestFilterObjects(unittest.TestCase):
    pass


class TestFilterCorrectSign(unittest.TestCase):
    pass


class TestFilterSigns(unittest.TestCase):
    pass


if __name__ == '__main__':
    unittest.main()
