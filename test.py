import unittest
from src.buoys import Buoy
from src.control_tasks import utils


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
    pass

class TestFilterSigns(unittest.TestCase):
    pass

if __name__ == '__main__':
    unittest.main()
