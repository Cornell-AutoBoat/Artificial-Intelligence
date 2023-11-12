import src.SFR as SFR
import src.tools.utils as utils
from src.tools.a_star import get_waypoints
from src.tools.path_processing import process, send_to_controls
from src.buoys import Buoy
import src.control_tasks.northern_passage as northern_passage
import numpy as np
import time
import rospy


def test_panama_canal():
    # Navigate through the two gates of red and green buoys
    rospy.loginfo("attempting panama canal")

    buoys, _ = utils.filter_objects(["red-buoy", "green-buoy"])

    waypoints = []
    if len(buoys) == 4:
        rospy.loginfo("all buoys seen immediately")
        mid_x1, mid_z1 = utils.get_extended_midpoint(buoys[0], buoys[1], t=1)
        mid_x2, mid_z2 = utils.get_extended_midpoint(buoys[2], buoys[3], t=3)
        waypoints.append(utils.map_to_global(mid_x1, mid_z1))
        waypoints.append(utils.map_to_global(mid_x2, mid_z2))
        path = process(waypoints)
        send_to_controls("path", path)
        while not SFR.pp_done:
            pass
    else:
        rospy.loginfo("can't see all buoys")
        seen = set()
        gates_passed = 0
        # completion criteria: 2 gates identified and passed through
        while gates_passed < 2:
            gates, seen = panama_canal_pivot(seen)
            if not gates[0, 0] or not gates[0, 1]:
                rospy.loginfo("FAIL")
                return
            else:
                waypoints = []
                for gate in gates:
                    b1, b2 = gate[0], gate[1]
                    if b1 and b2:
                        # calculate the midpoint in local coords
                        mid_x, mid_z = utils.get_extended_midpoint(b1, b2)
                        waypoints.append(utils.map_to_global(mid_x, mid_z))
                        gates_passed += 1
                path = process(waypoints)
                send_to_controls("path", path)
                while not SFR.pp_done:
                    pass
        send_to_controls("stop")
        rospy.loginfo("SUCCESS")


def test_northern_passage():
    # Test northern passage waypoint creation. Boat should loop around the blue buoy
    # after entering through the red and green gate buoys then navigate back to its starting position
    rospy.loginfo("attempting northern passage")

    buoys, _ = northern_passage.filter_buoys(SFR.objects)

    northern_passage.pivot()

    rospy.loginfo("found all buoys: (" + str(buoys[0].x) + ", " + str(buoys[0].z) + ")" + "(" + str(
        buoys[1].x) + ", " + str(buoys[1].z) + ")" + "(" + str(buoys[2].x) + ", " + str(buoys[2].z) + ")")

    waypoints = northern_passage.create_waypoints()

    path = process(waypoints)
    send_to_controls("path", path)
    while not SFR.pp_done:
        pass

    for wp in waypoints:
        rospy.loginfo("determined waypoint: " + str(wp))

    northern_passage.finish("SUCCESS")
    rospy.loginfo("done northern passage")


def test_beaching():
   # Test beaching task. There will be 2 die faces of different colors. Boat should
    # identify the correct color, move towards that sign (but not hit the side of
    # the pool), then back up
    rospy.loginfo("attempting to complete beaching")

    signs, s1 = utils.filter_signs()
    correct_sign, s2 = utils.filter_correct_sign()

    start = time.time()
    sL = 1500
    sR = 1500
    # Boat takes 2 pi seconds to pivot in a full circle at 1 rad/sec
    if signs.size < 2 and correct_sign.size != 1:
        send_to_controls("pivot_l")
        while signs.size < 2 and correct_sign.size != 1 and time.time() - start < 2*np.pi:
            signs, s1 = utils.filter_signs()
            correct_sign, s2 = utils.filter_correct_sign()
            time.sleep(0.5)
            send_to_controls("stop")

    send_to_controls("stop")

    for s in signs:
        rospy.loginfo("found sign: (" + str(s.x) + ", " + str(s.z) + ")")
    for s in correct_sign:
        rospy.loginfo("found correct sign: (" +
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
        rospy.loginfo("found sideSign: (" +
                      str(sideSign.x) + ", " + str(sideSign.z) + ")")
        rospy.loginfo("found correctSign: (" +
                      str(correct_sign[0].x) + ", " + str(correct_sign[0].z) + ")")
        waypoint = [utils.get_shifted_em(correct_sign[0], sideSign, -1)]

        rospy.loginfo("determined waypoint: " + str(waypoint))
        rospy.loginfo("eggs found: " + correct_sign[0].label[1])

        path = process(waypoint)
        send_to_controls("path", path)
        while not SFR.pp_done:
            pass

        send_to_controls("bwd")
        time.sleep(2)
        send_to_controls("stop")

        rospy.loginfo("done beaching")
    else:
        rospy.loginfo("Signs not seen")


def test_a_star_magellans_route(run=True):
    # Test A* functionality. Goal position should be a die face sign on the side
    # of the pool (but don't hit the side of the pool). There will be a buoy or
    # two in the way of the boat to the sign.
    # NOTE: might have to make object coords negative
    rospy.loginfo("going through mini magellans route")

    objects_map = [[0]*100 for _ in range(100)]
    objects, _ = utils.filter_objects(
        ['red-buoy', 'green-buoy', 'yellow-buoy', 'black-buoy'])
    goal_object, _ = utils.filter_objects(['Blue-Buoy'])
    goal = utils.map_to_global(goal_object[0].x, goal_object[0].z)

    for object in objects:
        rospy.loginfo("building map from object list")
        object_x, object_z = utils.map_to_global(object.x, object.z)
        object_row_index = int(object_x - SFR.tx) + 50
        object_col_index = int(object_z - SFR.tz) + 50
        if object_row_index < 100 and object_row_index >= 0 and object_col_index < 100 and object_col_index >= 0:
            objects_map[object_row_index][object_col_index] = 1
        else:
            rospy.loginfo("object out of range")

    rospy.loginfo("getting waypoints")
    waypoint_indices = get_waypoints(
        objects_map, (50, 50), (goal[0] - SFR.tx + 50, goal[1] - SFR.tz + 50))
    waypoint_global = []

    rospy.loginfo("converting waypoints to global coordinates")
    for index_x, index_z in waypoint_indices:
        global_x = SFR.tx + index_x - 50

        global_z = SFR.tz + index_z - 50

        waypoint_global.append([global_x, global_z])

    rospy.loginfo("going towards waypoints")
    rospy.loginfo(waypoint_global)
    if run:
        path = process([waypoint_global])
        send_to_controls("path", path)
        while not SFR.pp_done:
            pass

def explore_reef():
    pass


def execute():
    rospy.loginfo("starting task visualization")
    # test_panama_canal()
    # test_northern_passage()
    # test_beaching()
    # test_a_star_magellans_route()
    # test_explore_reef()
    SFR.execution_done = True
    rospy.loginfo("finished task visualization")
