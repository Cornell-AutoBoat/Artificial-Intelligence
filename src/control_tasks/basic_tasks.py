import src.SFR as SFR
import tools.utils as utils
from tools.a_star import get_waypoints
from tools.path_processing import process, send_to_controls
from src.buoys import Buoy
import src.control_tasks.northern_passage as northern_passage
import numpy as np
import time
import rospy


def test_auto_mode():
    print("here!")
    time.sleep(30)


def test_tmx():
    print("arrived")
    time.sleep(30)
    send_to_controls("fwd")
    time.sleep(1)
    send_to_controls("stop")


def test_send_signals_lin():
    # move straight at 1 m/s for 3 seconds
    print("start")
    send_to_controls("fwd")
    print("stop")
    send_to_controls("stop")


def test_send_signals_ang():
    # # pivot in place
    send_to_controls("pivot_l")
    # print("x: " + str(SFR.ang_vx) + "y: " + str(SFR.ang_vy) + "z: " + str(SFR.ang_vz))
    time.sleep(3)
    send_to_controls("stop")


def test_heading():
    # turn 180 degrees
    orig_heading = SFR.heading
    send_to_controls("pivot_r")
    while np.abs(SFR.heading - orig_heading) < np.pi:
        pass
    send_to_controls("stop")


def test_move_straight():
    waypoint = utils.map_to_global(0, 3)
    path = process([waypoint])
    send_to_controls("path", path)
    while not SFR.pp_done:
        pass
    send_to_controls("stop")


def test_move_side():
    waypoint = utils.map_to_global(2, 2)
    path = process([waypoint])
    send_to_controls("path", path)
    while not SFR.pp_done:
        pass
    send_to_controls("stop")


def test_circle_around():
    waypoint1 = utils.map_to_global(1.5, 1.5)
    waypoint2 = utils.map_to_global(0.5, 3)
    waypoint3 = utils.map_to_global(-0.5, 3)
    waypoint4 = utils.map_to_global(-1.5, 1.5)
    waypoint5 = utils.map_to_global(0, 0)
    path = process([waypoint1, waypoint2, waypoint3, waypoint4, waypoint5])
    send_to_controls("path", path)
    while not SFR.pp_done:
        pass
    send_to_controls("stop")


def test_hit_buoy():
    # Hit the red buoy
    rospy.loginfo("attempting to hit the red buoy")

    buoy, _ = utils.filter_objects(['red-buoy'])

    # Boat takes 2 pi seconds to pivot in a full circle at 1 rad/sec
    if not np.all(buoy):
        start = time.time()
        send_to_controls("pivot_r")
        while not np.all(buoy):
            if time.time() - start < 4*np.pi:
                rospy.loginfo("couldn't find buoy")
                send_to_controls("stop")
                return
            buoy, _ = utils.filter_buoys(['red-buoy'])
        send_to_controls("stop")
        if not np.all(buoy):
            rospy.loginfo("couldn't find buoy")
            return

    rospy.loginfo(
        "found buoy: (" + str(buoy[0].x) + ", " + str(buoy[0].z) + ")")

    waypoint = [utils.map_to_global(buoy[0].x, buoy[0].z)]

    rospy.loginfo("determined waypoint: " + str(waypoint))

    path = process([waypoint])
    send_to_controls("path", path)
    while not SFR.pp_done:
        pass
    send_to_controls("stop")

    rospy.loginfo("done hitting buoy")


def test_extended_buoy():
    # Navigate to a point 3 meters away from the red buoy
    rospy.loginfo("attempting extended buoy")

    buoy, _ = utils.filter_objects(["red-buoy"])
    print(buoy)

    if buoy.size == 0:
        start = time.time()
        send_to_controls("pivot_r")
        while (buoy.size == 0):
            if time.time() - start < 12:
                rospy.loginfo("couldn't find buoy")
                send_to_controls("stop")
                return
            buoy, _ = utils.filter_buoys(['red-buoy'])
        send_to_controls("stop")

    rospy.loginfo(
        "found buoy: (" + str(buoy[0].x) + ", " + str(buoy[0].z) + ")")

    ext_buoy = utils.get_extended_buoy(buoy[0], t=1)

    waypoint = [utils.map_to_global(ext_buoy[0], ext_buoy[1])]

    rospy.loginfo("determined waypoint: " + str(waypoint))

    path = process([waypoint])
    send_to_controls("path", path)
    while not SFR.pp_done:
        pass
    send_to_controls("stop")

    rospy.loginfo("done extended buoy")


def test_gate():
    gate = np.array([None, None])
    start = time.time()

    # Search left then search right. Conditions: a gate buoy has not yet been
    # identified and we haven't turned 90 degrees (pi/2 radians)
    buoys, s = utils.filter_objects(["green-column-buoy", "red-column-buoy"])
    print(buoys)
    if buoys.size < 2 or (buoys[0].label == buoys[1].label):
        send_to_controls("pivot_l")
        sL, sR = 1450, 1550
        while time.time() - start < 4:
            buoys, s = utils.filter_objects(["green-column-buoy", "red-column-buoy"], s)
            for b in buoys:
                if b.label == "red-buoy" and (not gate[0] or gate[0].z > b.z):
                    gate[0] = b
                elif not gate[1] or gate[1].z > b.z:
                    gate[1] = b
            # pivot left
        send_to_controls("stop")

        start = time.time()
        send_to_controls("pivot_r")
        sL, sR = 1550, 1450
        while time.time() - start < 8:
            buoys, s = utils.filter_objects(["green-column-buoy", "red-column-buoy"], s)
            for b in buoys:
                if b.label == "red-column-buoy" and (not gate[0] or gate[0].z > b.z):
                    gate[0] = b
                elif not gate[1] or gate[1].z > b.z:
                    gate[1] = b
            # pivot right
        send_to_controls("stop")

        # pivot to recenter
        start = time.time()
        send_to_controls("pivot_l")
        sL, sR = 1450, 1550
        while time.time() - start < 4:
            pass
            # pivot left
        send_to_controls("stop")

        if gate[0] == None or gate[1] == None:
            rospy.loginfo("couldn't find gate")
            return
    else:
        if buoys[0].label == "red-column-buoy":
            gate[0] = buoys[0]
            gate[1] = buoys[1]
        else:
            gate[1] = buoys[0]
            gate[0] = buoys[1]

    rospy.loginfo("found gate")

    mpt = utils.get_extended_midpoint(gate[0], gate[1])
    waypoint = [utils.map_to_global(mpt[0], mpt[1])]

    rospy.loginfo("determined waypoint: " + str(waypoint))

    path = process([waypoint])
    send_to_controls("path", path)
    while not SFR.pp_done:
        pass
    send_to_controls("stop")

    rospy.loginfo("done extended buoy")


def panama_canal_pivot(seen):
    # [[close red, close green], [far red, far green]]
    gates = np.array([[None, None], [None, None]])

    # Search left then search right. Conditions: a gate buoy has not yet been
    # identified and we haven't turned 90 degrees (pi/4 radians)
    for direction in ["l", "r", "r", "l"]:
        start = time.time()
        while (not gates[0, 0] or not gates[0, 1]) and time.time() - start < np.pi / 4:
            buoys, seen = utils.filter_buoys(
                ["green-buoy", "red-buoy"], seen)
            for b in buoys:
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
            send_to_controls("pivot_" + direction)
        send_to_controls("stop")
    return gates, seen


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


def test_a_star(run=True):
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


def execute():
    rospy.loginfo("starting basic taks")
    # test_kill()
    # test_tmx()
    # test_auto_mode()
    # test_send_signals_lin()
    # test_send_signals_ang()
    test_move_straight()
    # test_move_side()
    # test_circle_around()
    # test_hit_buoy()
    # test_gate()
    # test_extended_buoy()
    # test_panama_canal()
    # test_northern_passage()
    # test_beaching()
    # test_a_star()
    SFR.execution_done = True
    rospy.loginfo("finished basic tasks")
