import src.SFR as SFR
import src.tools.utils as utils
from src.tools.a_star import get_waypoints
from src.tools.path_processing import process, send_to_controls
from src.buoys import Buoy
import src.control_tasks.northern_passage as northern_passage
import numpy as np
import time
# import rospy


def panama_canal_pivot(seen):
    # [[close red, close green], [far red, far green]]
    gates = np.array([[None, None], [None, None]])

    # Search left then search right. Conditions: a gate buoy has not yet been
    # identified and we haven't turned 90 degrees (pi/4 radians)
    for direction in ["l", "r", "r", "l"]:
        start = time.time()
        while (not gates[0, 0] or not gates[0, 1]) and time.time() - start < np.pi / 4:
            buoys, seen = utils.filter_objects(["green-buoy", "red-buoy"], seen, sort_by='dist')
            for b in buoys:
                print(b.label)
                if b.z < 7.0:   # buoy is less than 7 meters away
                    if b.label == "red-buoy":
                        gates[0, 0] = b
                    else:
                        gates[0, 1] = b
                else:
                    if b.label == "red-buoy":
                        gates[1, 0] = b
                    else:
                        gates[1, 1] = b
        #     send_to_controls("pivot_" + direction)
        # send_to_controls("stop")
    return gates, seen


def test_panama_canal(fname):
    # Navigate through the two gates of red and green buoys
    print("attempting panama canal")

    buoys, _ = utils.filter_objects(["red-buoy", "green-buoy"], sort_by='dist')
    for b in buoys:
        print("label: " + b.label + ", " + "x: " + str(b.x) + ", y: " + str(b.y))

    waypoints = []
    if len(buoys) == 4:
        print("all buoys seen immediately")
        mid_x1, mid_z1 = utils.get_midpoint(buoys[0], buoys[1])
        print("mpt1: (" + str(mid_x1) + ", " + str(mid_z1) + ")")
        mid_x2, mid_z2 = utils.get_midpoint(buoys[2], buoys[3])
        print("mpt2: (" + str(mid_x2) + ", " + str(mid_z2) + ")")
        gmx1, gmz1 = utils.map_to_global(mid_x1, mid_z1)
        print("gmpt2: (" + str(gmx1) + ", " + str(gmz1) + ")")
        gmx2, gmz2 = utils.map_to_global(mid_x2, mid_z2)
        print("gmpt2: (" + str(gmx2) + ", " + str(gmz2) + ")")
        waypoints.append(utils.map_to_global(mid_x1, mid_z1))
        waypoints.append(utils.map_to_global(mid_x2, mid_z2))
        path = process(waypoints, fname)
        print("SUCCESS")
    else:
        print("FAILURE")


def test_panama_canal_driver():
    SFR.tx, SFR.ty = 2.0, 1.0
    SFR.heading = 3*np.pi / 4
    SFR.objects = np.array([Buoy("red-buoy", 2*np.sqrt(2), 1.5*np.sqrt(2)), 
                            Buoy("red-buoy", 4*np.sqrt(2), 1.5*np.sqrt(2)), 
                            Buoy("green-buoy", 2*np.sqrt(2), -np.sqrt(2)), 
                            Buoy("green-buoy", 4*np.sqrt(2), -np.sqrt(2))])
    test_panama_canal("nav_channel_neg_y.png")

    SFR.tx, SFR.ty = 2.0, -3.0
    SFR.heading = np.pi / 6
    SFR.objects = np.array([Buoy("red-buoy",-2, 2), 
                            Buoy("red-buoy", -3, 4), 
                            Buoy("green-buoy", 0, 1), 
                            Buoy("green-buoy", -2, 6)])
    test_panama_canal("nav_channel_neg_x.png")

    SFR.tx, SFR.ty = 1.0, 1.0
    SFR.heading = 2*np.pi / 5
    SFR.objects = np.array([Buoy("red-buoy", -0.5, 5), 
                            Buoy("red-buoy", -2, 2), 
                            Buoy("green-buoy", 0.5, 1), 
                            Buoy("green-buoy", 2, 5)])
    test_panama_canal("nav_channel_simple_tilted.png")

    SFR.tx, SFR.ty = 0.0, 0.0
    SFR.heading = np.pi / 2
    SFR.objects = np.array([Buoy("red-buoy", -1, 2), 
                            Buoy("green-buoy", 1, 4), 
                            Buoy("red-buoy", -1, 4),
                            Buoy("green-buoy", 1, 2)])
    test_panama_canal("nav_channel_simple_aligned.png")


def test_northern_passage(fname):
    # Test northern passage waypoint creation. Boat should loop around the yellow buoy
    # after entering through the red and green gate buoys then navigate back to its starting position
    print("attempting northern passage")

    buoys, _ = northern_passage.filter_buoys(SFR.objects)

    northern_passage.pivot()

    print("found all buoys: (" + str(buoys[0].x) + ", " + str(buoys[0].z) + ")" + "(" + str(
        buoys[1].x) + ", " + str(buoys[1].z) + ")" + "(" + str(buoys[2].x) + ", " + str(buoys[2].z) + ")")

    waypoints = northern_passage.create_waypoints()

    path = process(waypoints, fname)

    for wp in waypoints:
        print("determined waypoint: " + str(wp))

    northern_passage.finish("SUCCESS")
    print("done northern passage")


def test_northern_passage_driver():
    pass


def test_beaching(fname):
   # Test beaching task. There will be 2 die faces of different colors. Boat should
    # identify the correct color, move towards that sign (but not hit the side of
    # the pool), then back up
    print("attempting to complete beaching")

    signs, s1 = utils.filter_signs()
    correct_sign, s2 = utils.filter_correct_sign()

    start = time.time()
    # Boat takes 2 pi seconds to pivot in a full circle at 1 rad/sec
    if signs.size < 2 and correct_sign.size != 1:
        while signs.size < 2 and correct_sign.size != 1 and time.time() - start < 2*np.pi:
            signs, s1 = utils.filter_signs()
            correct_sign, s2 = utils.filter_correct_sign()
            time.sleep(0.5)

    for s in signs:
        print("found sign: (" + str(s.x) + ", " + str(s.z) + ")")
    for s in correct_sign:
        print("found correct sign: (" +
                      str(s.x) + ", " + str(s.z) + ")")

    if signs.size > 1 and correct_sign.size > 0:
        targetX = correct_sign[0].x
        sideIndex = -1
        i = 0
        while i < signs.size:
            if signs[i].label[0] != SFR.sign_color:
                # x distance between sign at index i and target sign
                iDifference = np.absolute(signs[i].x - targetX)
                if sideIndex == -1:
                    sideIndex = i
                elif np.absolute(signs[sideIndex].x - targetX) > iDifference:
                    sideIndex = i
            i += 1
        sideSign = signs[sideIndex]
        print("found sideSign: (" +
                      str(sideSign.x) + ", " + str(sideSign.z) + ")")
        print("found correctSign: (" +
                      str(correct_sign[0].x) + ", " + str(correct_sign[0].z) + ")")
        waypoint = [utils.get_shifted_em(correct_sign[0], sideSign, -1)]

        print("determined waypoint: " + str(waypoint))
        print("eggs found: " + correct_sign[0].label[1])

        path = process(waypoint, fname)

        send_to_controls("bwd")
        time.sleep(2)
        send_to_controls("stop")

        print("done beaching")
    else:
        print("Signs not seen")


def test_beaching(fname):
    pass


def test_a_star_magellans_route(fname, run=True):
    # Test A* functionality. Goal position should be a die face sign on the side
    # of the pool (but don't hit the side of the pool). There will be a buoy or
    # two in the way of the boat to the sign.
    # NOTE: might have to make object coords negative
    print("going through mini magellans route")

    objects_map = [[0]*100 for _ in range(100)]
    objects, _ = utils.filter_objects(
        ['red-buoy', 'green-buoy', 'yellow-buoy', 'black-buoy'])
    goal_object, _ = utils.filter_objects(['Blue-Buoy'])
    goal = utils.map_to_global(goal_object[0].x, goal_object[0].z)

    for object in objects:
        print("building map from object list")
        object_x, object_z = utils.map_to_global(object.x, object.z)
        object_row_index = int(object_x - SFR.tx) + 50
        object_col_index = int(object_z - SFR.tz) + 50
        if object_row_index < 100 and object_row_index >= 0 and object_col_index < 100 and object_col_index >= 0:
            objects_map[object_row_index][object_col_index] = 1
        else:
            print("object out of range")

    print("getting waypoints")
    waypoint_indices = get_waypoints(
        objects_map, (50, 50), (goal[0] - SFR.tx + 50, goal[1] - SFR.tz + 50))
    waypoint_global = []

    print("converting waypoints to global coordinates")
    for index_x, index_z in waypoint_indices:
        global_x = SFR.tx + index_x - 50

        global_z = SFR.tz + index_z - 50

        waypoint_global.append([global_x, global_z])

    print("going towards waypoints")
    print(waypoint_global)
    if run:
        path = process([waypoint_global], fname)


def test_a_star_magellans_route_driver():
    pass

def test_explore_reef(fname):
    pass

def test_explore_reef_driver():
    pass


def execute():
    print("starting task visualization")
    test_panama_canal_driver()
    # test_northern_passage_driver()
    # test_beaching_driver()
    # test_a_star_magellans_route_driver()
    # test_explore_reef_driver()
    SFR.execution_done = True
    print("finished task visualization")
