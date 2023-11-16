import unittest
from src.buoys import Buoy
from src import SFR
import numpy as np
from src.tools import utils


class TestSeen(unittest.TestCase):
    def test_one_object_true(self):

        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 0.0, 0.0)
        s = set([o1])
        self.assertEqual(utils.seen(o1, s), True,
                         "Could not find buoy in list")

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
        self.assertEqual(utils.seen(o1, s), True,
                         "Could not find buoy in list")

    def test_same_label_diff_coords(self):

        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 0.0, 1.0)
        o2 = Buoy("red-buoy", 0.0, 3.0)
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), False,
                         "Wrongly identified buoy based on coordinates")

    def test_same_label_close_coords(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 0.0, 1.0)
        o2 = Buoy("red-buoy", 0.0, 1.2)
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), True,
                         "Couldn't identify that buoys were the same")

    def test_same_label_close_coords_mult_axes(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-buoy", 1.0, 2.0)
        o2 = Buoy("red-buoy", 1.5, 2.5)
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), True,
                         "Couldn't identify that buoys were the same")

    def test_seen_unaligned_frames(self):
        SFR.heading = 3 * np.pi / 4
        SFR.tx, SFR.ty = 4, 4
        o1 = Buoy("red-buoy", 5*np.sqrt(2)/2, 3*np.sqrt(2)/2)  # local
        o2 = Buoy("red-buoy", 5.0, 8.0)  # global
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), True,
                         "Couldn't identify that buoys were the same")

        o1 = Buoy("red-buoy", 0.0, 0.0, 1.0)
        o2 = Buoy("red-buoy", 0.0, 0.0, 3.0)
        s = [o2]
        self.assertEqual(utils.seen(o1, s), False,
                         "Wrongly identified buoy based on coordinates")

    def test_not_seen_unaligned_frames(self):
        SFR.heading = 3 * np.pi / 4
        SFR.tx, SFR.ty = 4, 4
        o1 = Buoy("red-buoy", 5.0, 8.0)  # local
        o2 = Buoy("red-buoy", 5.0, 8.0)  # global
        s = set([o2])
        self.assertEqual(utils.seen(o1, s), False,
                         "Couldn't identify that buoys were the same")


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
    SFR.heading = np.pi/2 

    def test_basic_midpoint_functionality(self):
        o1 = Buoy("red-buoy", 0.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 2.0, 0.0)
        x = -1
        y = 1.5
        self.assertEqual(utils.get_extended_midpoint(
            o1, o2, t=1), [x, y], "did not find midpoint")
    
    SFR.heading = 0
    
    
    def test_heading_on_basic_midpoint_functionality(self): 
        o1 = Buoy("red-buoy", 0.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 2.0, 0.0)
        x = -1
        y = 1.5
        self.assertEqual(utils.get_extended_midpoint(
            o1, o2, t=1), [x, y], "did not find midpoint")
    


    def testing_the_same_point(self):
        o1 = Buoy("red-buoy", 0.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 1.0, 0.0)
        x = -1
        y = 1
        self.assertEqual(utils.get_extended_midpoint(
            o1, o2, t=1), [x, y], "same point")

    SFR.tx = 1
    SFR.tz = 0
    SFR.ty = 1

    def test_of_two_positive_points(self):
        o1 = Buoy("red-buoy", 1.0, 5.0, 0.0)
        o2 = Buoy("green-buoy", 5.0, 1.0, 0.0)
        x = 4
        y = 4
        self.assertEqual(utils.get_extended_midpoint(
            o1, o2, t=1), [x, y], "same point")

    def test_negative_and_positive_points(self):
        o1 = Buoy("red-buoy", -1.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 1.0, 1.0, 0.0)
        x = 0
        y = 1
        self.assertEqual(utils.get_extended_midpoint(
            o1, o2, t=0), [x, y], "same point")

    def test_all_negative_points(self):
        o1 = Buoy("red-buoy", -2.0, -2.0, 0.0)
        o2 = Buoy("green-buoy", -3.0, -2.0, 0.0)
        x = -2.5
        y = -3.0
        self.assertEqual(utils.get_extended_midpoint(
            o1, o2, t=-1), [x, y], "same point")


class TestGetShiftedExtendedMidpoint(unittest.TestCase):
    SFR.tx = 0
    SFR.tz = 0
    SFR.ty = 0
    SFR.heading = np.pi/2 

   # o1, o2
    def test_basic_shifted_functionality(self):
        o1 = Buoy("red-buoy", 1.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 1.0, 3.0, 0.0)
        x = 2
        y = 1
        self.assertEqual(utils.get_shifted_em(o1, o2, t=-1),
                         [x, y], "did not find midpoint")
    
    SFR.heading = 0
    
    def test_heading_effect_on_shifted_midpoint(self):
        o1 = Buoy("red-buoy", 1.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 1.0, 3.0, 0.0)
        x = 2
        y = 1
        self.assertEqual(utils.get_shifted_em(o1, o2, t=-1),
                         [x, y], "did not find midpoint")
    
    SFR.heading = np.pi/2 

    
    def test_all_positive_points_shifted_functionality(self):
        o1 = Buoy("red-buoy", 1.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 1.0, 3.0, 0.0)
        x = 2
        y = 3
        self.assertEqual(utils.get_shifted_em(o2, o1, t=-1),
                         [x, y], "did not find midpoint")

    def test_negative_and_positive_points_shifted_functionality(self):
        o1 = Buoy("red-buoy", -3.0, 3.0, 0.0)
        o2 = Buoy("green-buoy", -3.0, -3.0, 0.0)
        x = -6
        y = 3
        self.assertEqual(utils.get_shifted_em(o1, o2, t=3),
                         [x, y], "did not find midpoint")

    def test_same_point_shifted_functionality(self):
        o1 = Buoy("red-buoy", 0.0, 1.0, 0.0)
        o2 = Buoy("green-buoy", 0.0, 1.0, 0.0)
        x = 0
        y = 2
        self.assertEqual(utils.get_shifted_em(
            o1, o2, t=1), [x, y], "same point")


class TestGetExtendedBuoy(unittest.TestCase):

    SFR.tx = 0.0
    SFR.ty = 0.0
    SFR.tz = 0.0
    SFR.heading = 0.0

    def test_one(self):
        o1 = Buoy("red-buoy", 3.0, 4.0, 0.0)
        self.assertEqual(utils.get_extended_buoy(
            o1, 5), [0.0, 0.0])

    def test_zero_t(self):
        o1 = Buoy("red-buoy", 1.0, 1.0, 0.0)
        self.assertEqual(utils.get_extended_buoy(o1, 0), [1.0, 1.0])

    def test_negative_t(self):
        o1 = Buoy("red-buoy", 6.0, 8.0, 0.0)
        self.assertEqual(utils.get_extended_buoy(o1, -10), [12.0, 16.0])

    def test_large_t(self):
        o1 = Buoy("red-buoy", 5.0, 12.0, 0.0)
        self.assertEqual(utils.get_extended_buoy(o1, 39),
                         [-10, -24])

    def test_negative_coordinates(self):
        o1 = Buoy("negative-buoy", -3.0, -4.0, 0.0)
        self.assertEqual(utils.get_extended_buoy(o1, 20),
                         [9, 12])


class TestGetYaw(unittest.TestCase):
    pass


class TestFilterObjects(unittest.TestCase):
    pass


class TestFilterCorrectSign(unittest.TestCase):
    """
    Function returns:
        objs: sorted numpy array of correct sign
        seen: seen objects
    """
    def test_empty_set(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        SFR.objects = []
        SFR.sign_color = "r"
        objs, seen = utils.filter_correct_sign()
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertEqual(seen, set())
    def test_one_sign_true(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-sign", 0.0, 0.0, 0.0)
        SFR.objects = [o1]
        SFR.sign_color = "r"
        o1g = utils.map_to_global_Buoy(o1)
        objs, seen = utils.filter_correct_sign(set())
        self.assertTrue(np.array_equal(objs, np.array([o1])), "testing first return value")
        self.assertTrue(set_equality(seen, {o1g}), "didn't see correct buoy")
    def test_one_sign_false(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-sign", 0.0, 0.0, 0.0)
        SFR.objects = [o1]
        SFR.sign_color = "g"
        objs, seen = utils.filter_correct_sign(set())
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        o1g = utils.map_to_global_Buoy(o1)
        self.assertTrue(set_equality(seen, {o1g}), "didn't see correct buoy")
    def test_two_signs_seen(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o2 = Buoy("green-sign", 0.0, 0.0, 0.0)
        o3 = Buoy("red-sign", 2.0, 0.0, 0.0)
        o3g = utils.map_to_global_Buoy(o3)
        SFR.objects = [o1,o2]
        SFR.sign_color = "g"
        objs, seen = utils.filter_correct_sign({o3g})
        self.assertTrue(np.array_equal(objs, [o2]), "testing first return value")
        o1g = utils.map_to_global_Buoy(o1)
        o2g = utils.map_to_global_Buoy(o2)
        self.assertTrue(set_equality(seen, {o3g, o1g, o2g}), "didn't see correct buoy")
       

    def test_multiple_signs(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("green-sign", 0.0, 0.0, 0.0)
        o2 = Buoy("green-sign", -2.0, 0.0, 0.0)
        o3 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o4 = Buoy("green-sign", -3.0, 0.0, 0.0)
        SFR.objects = [o1,o2,o3,o4]
        SFR.sign_color = "r"
        objs, seen = utils.filter_correct_sign(set())
        self.assertTrue(np.array_equal(objs, np.array([o3])), "testing first return value")
        self.assertTrue(set_equality(seen, {o1, o2, o3, o4}), "didn't see correct buoy")

    def test_multiple_signs_false(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("green-sign", 0.0, 0.0, 0.0)
        o2 = Buoy("green-sign", -2.0, 0.0, 0.0)
        o3 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o4 = Buoy("green-sign", -3.0, 0.0, 0.0)
        SFR.objects = [o1,o2,o3,o4]
        SFR.sign_color = "g"
        objs, seen = utils.filter_correct_sign(set())
        self.assertTrue(np.array_equal(objs, np.array([])), "testing first return value")
        self.assertTrue(set_equality(seen, {o1, o2, o3, o4}), "didn't see correct buoy")

class TestFilterSigns(unittest.TestCase):
    def test_empty_set(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        SFR.objects = np.array([])
        objs, seen = utils.filter_signs(set())
        self.assertTrue(np.array_equal(objs, np.array([])), "no buoy in list")
        self.assertEqual(seen, set())

    def test_one_sign(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-sign", 0.0, 0.0, 0.0)
        o1g = utils.map_to_global_Buoy(o1)
        SFR.objects = [o1]
        print("calling")
        objs, seen = utils.filter_signs(set())
        self.assertTrue(np.array_equal(objs, np.array([o1])), "need more than 1 buoy to filter")
        print(seen)
        #print({o1}) 

        self.assertTrue(set_equality(seen,{o1}), "wrongly identified buoy")
        
    def test_two_signs(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        
        o1 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o2 = Buoy("green-sign", 0.0, 0.0, 0.0)
        SFR.objects = [o2,o1]
        objs, s = utils.filter_signs(set())
        self.assertTrue(np.array_equal(objs, np.array(SFR.objects)), "wrongly filtered buoy")
        self.assertTrue(set_equality(s,{o2, o1}), "wrongly identified buoy")

    def test_two_signs_seen(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o2 = Buoy("green-sign", 8.0, 0.0, 0.0)
        o3 = Buoy("green-sign", -2.0, 0.0, 0.0)
        o4 = Buoy("red-sign", 0.0, 0.0, 0.0)
        SFR.objects = [o3,o1,o2]
        objs, s = utils.filter_signs({o4})
        self.assertTrue(np.array_equal(objs, np.array(SFR.objects)), "wrongly filtered buoys")
        o1g = utils.map_to_global_Buoy(o1)
        o2g = utils.map_to_global_Buoy(o2)
        o3g = utils.map_to_global_Buoy(o3)
        o4g = utils.map_to_global_Buoy(o4)
        self.assertTrue(set_equality(s,{o4g, o3g, o1g, o2g}), "seen buoys are not the same")        

    def test_multiple_signs(self):
        SFR.heading = np.pi / 2
        SFR.tx, SFR.ty = 0.0, 0.0
        o1 = Buoy("red-sign", 4.0, 0.0, 0.0)
        o2 = Buoy("green-sign", 0.0, 0.0, 0.0)
        o3 = Buoy("green-sign", -2.0, 0.0, 0.0)
        o4 = Buoy("green-sign", -3.0, 0.0, 0.0)
        SFR.objects = [o4,o3,o2,o1]
        objs, s = utils.filter_signs(set())
        o1g = utils.map_to_global_Buoy(o1)
        o2g = utils.map_to_global_Buoy(o2)
        o3g = utils.map_to_global_Buoy(o3)
        o4g = utils.map_to_global_Buoy(o4)
        self.assertTrue(np.array_equal(objs, np.array(SFR.objects)), "wrongly filtered buoys")
        self.assertTrue(set_equality(s,{o4g, o3g, o2g, o1g}), "seen buoys are not the same")

def set_equality(set1, set2):
    result = True
    l1 = sorted(list(set1), key=lambda b: b.x)
    l2 = sorted(list(set2), key=lambda b: b.x)
    # sort 
    if len(l1) != len(l2): result = False
    else:
        for i in range(len(l1)):
            result = result and (l1[i].x == l2[i].x and l1[i].y == l2[i].y and l1[i].label == l2[i].label)
    return result


if __name__ == '__main__':
    unittest.main()
