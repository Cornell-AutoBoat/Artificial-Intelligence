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
    pass


class TestGetExtendedMidpoint(unittest.TestCase):
    
    SFR.tx = 0
    SFR.tz = 0 
    SFR.ty = 0

    def test_basic_midpoint_functionality(self):
        o1 = Buoy("red-buoy", 0.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 2.0, 0.0)
        x = -1
        y = 1.5
        self.assertEqual(utils.get_extended_midpoint(o1, o2, t = 1), [x, y], "did not find midpoint" )


    def test_midpoint_same_point(self): 
        o1 = Buoy("red-buoy", 0.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 1.0, 0.0 )
        x = -1
        y = 1
        self.assertEqual(utils.get_extended_midpoint(o1, o2, t=1), [x, y], "same point")

    SFR.tx = 1
    SFR.tz = 0 
    SFR.ty = 1

    def test_midpoint_same_point(self): 
        o1 = Buoy("red-buoy", 1.0, 5.0, 0.0)
        o2 = Buoy("green-buoy", 5.0, 1.0, 0.0 )
        x = 4
        y = 4
        self.assertEqual(utils.get_extended_midpoint(o1, o2, t=1), [x, y], "same point")


    def test_midpoint_same_point(self): 
        o1 = Buoy("red-buoy", -1.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 1.0, 1.0, 0.0 )
        x = 0
        y = 1
        self.assertEqual(utils.get_extended_midpoint(o1, o2, t=0), [x, y], "same point")


    def test_midpoint_same_point(self): 
        o1 = Buoy("red-buoy", -2.0, -2.0, 0.0)
        o2 = Buoy("green-buoy", -3.0, -2.0, 0.0 )
        x = -2.5
        y = -3.0
        self.assertEqual(utils.get_extended_midpoint(o1, o2, t=-1), [x, y], "same point")



class TestGetShiftedExtendedMidpoint(unittest.TestCase):
    SFR.tx = 0
    SFR.tz = 0 
    SFR.ty = 0

   #o1, o2
    def test_basic_shifted_functionality_2points(self):
        o1 = Buoy("red-buoy", 1.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 1.0, 3.0, 0.0)
        x = 2
        y = 1
        self.assertEqual(utils.get_shifted_em(o1, o2, t = -1), [x, y], "did not find midpoint" )

    #o2,01
    def test_basic_shifted_functionality_2points(self):
        o1 = Buoy("red-buoy", 1.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 1.0, 3.0, 0.0)
        x = 2
        y = 3
        self.assertEqual(utils.get_shifted_em(o2, o1, t = -1), [x, y], "did not find midpoint" )

    def test_basic_shifted_functionality_2points(self):
        o1 = Buoy("red-buoy", -3.0, 3.0, 0.0)
        o2 = Buoy("green-buoy", -3.0, -3.0, 0.0)
        x = -6
        y = 3
        self.assertEqual(utils.get_shifted_em(o1, o2, t =3), [x, y], "did not find midpoint" )

    def test_midpoint_same_point(self): 
        o1 = Buoy("red-buoy", 0.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 1.0, 0.0 )
        x = 0
        y = 2
        self.assertEqual(utils.get_shifted_em(o1, o2, t=1), [x, y], "same point")


class TestGetExtendedBuoy(unittest.TestCase):
    pass


class TestGetYaw(unittest.TestCase):
    pass


class TestFilterObjects(unittest.TestCase):
    pass


class TestFilterCorrectSign(unittest.TestCase):
    def test_empty_set(self):
        SFR.objects = []
        objs, seen = utils.filter_correct_sign()
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertEqual(seen, [], "testing second return value")
    def test_one_sign(self):
        o1 = Buoy("red-buoy", 0.0, 0.0, 0.0)
        SFR.objects = [o1]
        SFR.sign_color = "r"
        objs, seen = utils.filter_correct_sign()
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertEqual(seen, [], "testing second return value")
    def test_two_signs(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        SFR.objects = [o1,o2]
        SFR.sign_color = "g"
        objs, seen = utils.filter_correct_sign()
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertEqual(seen, [], "testing second return value")
    def test_multiple_signs(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        o3 = Buoy("green-buoy", -2.0, 0.0, 0.0)
        o4 = Buoy("green-buoy", -3.0, 0.0, 0.0)
        SFR.objects = [o1,o2,o3,o4]
        SFR.sign_color = "r"
        objs, seen = utils.filter_correct_sign()
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertEqual(seen, [], "testing second return value")
    def test_multiple_signs_seen(self):
        o1 = Buoy("red-buoy", 4.0, 0.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 0.0, 0.0)
        o3 = Buoy("green-buoy", -2.0, 0.0, 0.0)
        o4 = Buoy("green-buoy", -3.0, 0.0, 0.0)
        SFR.objects = [o1,o2,o3,o4]
        SFR.sign_color = "g"
        objs, seen = utils.filter_correct_sign()
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertEqual(seen, [], "testing second return value")
        

class TestFilterSigns(unittest.TestCase):
    def test_empty_set(self):
        SFR.objects = np.array([])
        objs, seen = utils.filter_signs()
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertEqual(seen, [], "testing second return value")
    def test_one_sign(self):
        o1 = Buoy("red-sign", 0.0, 0.0, 0.0)
        SFR.objects = [o1]
        objs, seen = utils.filter_signs()
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertEqual(seen, [], "testing second return value")
    def test_two_signs(self):
        o1 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o2 = Buoy("green-sign", 0.0, 0.0, 0.0)
        SFR.objects = [o2,o1]
        objs, seen = utils.filter_signs()
        self.assertTrue(np.array_equal(objs, np.array(SFR.objects)), "testing first return value")
        self.assertTrue(np.array_equal(seen, np.array(SFR.objects)), "testing second return value")
    def test_two_signs_seen(self):
        o1 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o2 = Buoy("green-sign", 8.0, 0.0, 0.0)
        o3 = Buoy("green-sign", -2.0, 0.0, 0.0)
        SFR.objects = [o3,o1,o2]
        objs, seen = utils.filter_signs()
        self.assertTrue(np.array_equal(objs, np.array(SFR.objects)), "testing first return value")
        self.assertTrue(np.array_equal(seen, np.array(SFR.objects)), "testing second return value")
    def test_multiple_signs(self):
        o1 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o2 = Buoy("green-sign", 0.0, 0.0, 0.0)
        o3 = Buoy("green-sign", -2.0, 0.0, 0.0)
        o4 = Buoy("green-sign", -3.0, 0.0, 0.0)
        SFR.objects = [o4,o3,o2,o1]
        objs, seen = utils.filter_signs()
        self.assertTrue(np.array_equal(objs, np.array(SFR.objects)), "testing first return value")
        self.assertTrue(np.array_equal(seen, np.array(SFR.objects)), "testing second return value")
    def test_duplicates(self):
        o1 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o2 = Buoy("green-sign", -2.0, 0.0, 0.0)
        o3 = Buoy("green-sign", -2.0, 0.0, 0.0)
        o4 = Buoy("green-sign", -3.0, 0.0, 0.0)
        SFR.objects = [o4,o2,o3,o1]
        objs, seen = utils.filter_signs()
        self.assertTrue(np.array_equal(objs, np.array(SFR.objects)), "testing first return value")
        self.assertTrue(np.array_equal(seen, np.array(SFR.objects)), "testing second return value")
        SFR.objects = [o4,o3,o2,o1]
        objs, seen = utils.filter_signs()
        self.assertTrue(np.array_equal(objs, np.array(SFR.objects)), "testing first return value")
        self.assertTrue(np.array_equal(seen, np.array(SFR.objects)), "testing second return value")



if __name__ == '__main__':
    unittest.main()
