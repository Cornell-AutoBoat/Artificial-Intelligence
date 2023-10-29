import unittest
from src.buoys import Buoy
from src import SFR
from src.control_tasks import utils
import numpy as np


class TestSeen(unittest.TestCase):
    def test_one_object_true(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 0.0)
        s = [o1]
        self.assertEqual(utils.seen(o1, s), True, "Could not find buoy in list")

    def test_one_object_false(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        s = [o2]
        self.assertEqual(utils.seen(o1, s), False, "Wrongly identified buoy")

    def test_simple_mult_object_true(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        o3 = Buoy("blue-buoy", 0.0, 0.0, 0.0)
        s = [o1, o2, o3]
        self.assertEqual(utils.seen(o1, s), True, "Could not find buoy in list")

    def test_simple_mult_object_false(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        o3 = Buoy("blue-buoy", 0.0, 0.0, 0.0)
        s = [o2, o3]
        self.assertEqual(utils.seen(o1, s), False, "Wrongly identified buoy")

    def test_same_label_diff_coords(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 1.0)
        o2 = Buoy("red-buoy", 0.0, 0.0, 3.0)
        s = [o2]
        self.assertEqual(utils.seen(o1, s), False, "Wrongly identified buoy based on coordinates")

    def test_same_label_close_coords(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 1.0)
        o2 = Buoy("red-buoy", 0.0, 0.0, 1.2)
        s = [o2]
        self.assertEqual(utils.seen(o1, s), True, "Couldn't identify that buoys were the same")

    def test_same_label_close_coords_mult_axes(self):
        o1 = Buoy("red-buoy", 1.0, 0.0, 2.0)
        o2 = Buoy("red-buoy", 1.5, 0.0, 2.5)
        s = [o2]
        self.assertEqual(utils.seen(o1, s), True, "Couldn't identify that buoys were the same")

class TestMapToGlobal(unittest.TestCase):
    pass

class TestGetMidpoint(unittest.TestCase):
    pass

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
    def test_empty_set(self):
        SFR.objects = []
        self.assertEqual(utils.filter_correct_sign(), ([], []), "Testing for the empty set")
    def test_one_sign(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 0.0)
        SFR.objects = [o1]
        SFR.sign_color = "red"
        self.assertEqual(utils.filter_correct_sign(), ([o1], [o1]), "Testing for one sign")
    def test_two_signs(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        SFR.objects = [o1,o2]
        SFR.sign_color = "green"
        self.assertEqual(utils.filter_correct_sign(), ([o1], [o1]), "Testing for two different signs")
    def test_multiple_signs(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        o3 = Buoy("green-buoy", -2.0, 0.0, 0.0)
        o4 = Buoy("green-buoy", -3.0, 0.0, 0.0)
        SFR.objects = [o1,o2,o3,o4]
        SFR.sign_color = "red"
        self.assertEqual(utils.filter_correct_sign(), ([o1], [o1]), "Testing for mutliple different signs")
    def test_multiple_signs_seen(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        o3 = Buoy("green-buoy", -2.0, 0.0, 0.0)
        o4 = Buoy("green-buoy", -3.0, 0.0, 0.0)
        SFR.objects = [o1,o2,o3,o4]
        SFR.sign_color = "green"
        self.assertEqual(utils.filter_correct_sign([o1]), ([o2], [o1, o2]), "Testing for mutliple different signs with previously seen")

class TestFilterSigns(unittest.TestCase):
    def test_empty_set(self):
        SFR.objects = []
        self.assertEqual(utils.filter_signs(), [], "Testing for the empty set")
    def test_one_sign(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 0.0)
        SFR.objects = [o1]
        self.assertEqual(utils.filter_signs(), ([], [o1]), "Testing for one sign")
    def test_two_signs(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        SFR.objects = [o2,o1]
        self.assertEqual(utils.filter_signs(), ([o1, o2], [o1, o2]), "Testing for two signs")
    def test_two_signs_seen(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 8.0, 0.0, 0.0)
        o3 = Buoy("green-buoy", -2.0, 0.0, 0.0)
        SFR.objects = [o2,o1]
        self.assertEqual(utils.filter_signs([o3]), ([o1, o2], [o3, o1, o2]), "Testing for two signs with previously seen")
    def test_multiple_signs(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        o3 = Buoy("green-buoy", -2.0, 0.0, 0.0)
        o4 = Buoy("green-buoy", -3.0, 0.0, 0.0)
        SFR.objects = [o2,o3,o1,o4]
        self.assertEqual(utils.filter_signs(), ([o4, o3, o2, o1], [o4, o3, o2, o1]), "Testing for multiple signs")

if __name__ == '__main__':
    unittest.main()
