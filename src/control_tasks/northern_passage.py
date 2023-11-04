"""
This file contains the software for completing the Northern Passage (previously
Snack Run) Task.
"""
import src.SFR as SFR
import numpy as np
import tools.utils as utils
import src.path_execution.pure_pursuit as pure_pursuit
from src.modes.tasks_enum import Task
from src.modes.movement_modes_enum import Mode
import src.path_execution.thruster_utils as thruster_utils
import rospy
import math


def filter_buoys(objects):
    # filter buoys array to contain valid buoys: must be green or red or blue

    # buoys contains only green, red, blue buoys, yellow buoy
    buoys = np.array(list(filter(lambda b: b.label == "green-buoy" or b.label ==
                                 "red-buoy" or b.label == "blue-buoy", b.label == 'yellow-buoy', objects)))

    # orders buoys by blue, green, red, yellow
    ordered_buoys = sorted(buoys, key=lambda o: o.label)

    return ordered_buoys


def pivot():
    # pivot until 3 buoys are seen
    buoys = filter_buoys(SFR.objects)

    # when less than 3 buoys are seen
    while (len(buoys) < 3):
        # when 2 buoys are seen
        # if a green and red buoy are seen
        if buoys[0].label == "green-buoy" and buoys[1].label == "red-buoy":
            sL, sR = pure_pursuit.execute(
                utils.get_extended_midpoint(buoys[0], buoys[1], -1.75))
            buoys = filter_buoys(SFR.objects)
            thruster_utils.break_thrusters(sL, sR)
        # if a blue and red buoy are seen, but not a green
        if buoys[0].label == "blue-buoy" and buoys[1].label == "red-buoy":
            sL, sR = 1450, 1550
            thruster_utils.sendValue(sL, sR)
            start = time.time()
            while (not (buoys[0] == "blue-buoy" and buoys[1] == "green-buoy" and buoys[2] == "red-buoy")):
                buoys = filter_buoys(SFR.objects)
                if time.time() - start > 20:
                    thruster_utils.break_thrusters(sL, sR)
                    finish("FAILURE")
            thruster_utils.break_thrusters(sL, sR)
            sL, sR = pure_pursuit.execute(
                utils.get_extended_midpoint(buoys[0], buoys[1], -1.75))
            thruster_utils.break_thrusters(sL, sR)
            buoys = filter_buoys(SFR.objects)

            # last case?
            # if a yellow and red buoy are seen, but not a green
    # no buoys seen
    if len(buoys) < 3:
        finish("FAILURE")


def get_to_challenge():
    # pivot before transitioning to the northern passage task
    buoys = filter_buoys(SFR.objects)
    # pivot right initially to avoid seeing the red buoy from the magellans route task
    sL, sR = 1450, 1550
    thruster_utils.sendValue(sL, sR)
    start = time.time()
    while (buoys[0].label != "red-buoy" or buoys[0].label != "green-buoy"):
        buoys = filter_buoys(SFR.objects)
        if time.time() - start > 15:
            thruster_utils.break_thrusters(sL, sR)
            finish("FAILURE")
    thruster_utils.break_thrusters(sL, sR)
    pure_pursuit.execute(utils.get_extended_buoy(buoys[0], t=3))


def create_waypoints():
    # list of waypoints for the boat (6 total)
    waypoints = []
    buoys = filter_buoys(SFR.objects)
    buoy_radius = 0.184
    boat_radius = 0.381
    safety1 = 0.3  # placeholder value
    safety2 = 1.5
    j = buoys[0].x
    k = buoys[0].z
    t = 2  # placeholder value for how far away we want the boat to be from the blue buoy
    if (buoys[1].z != buoys[2].z):
        a = (buoys[1].z - buoys[2].z)/(buoys[1].x - buoys[2].x)
        b = buoys[0].z - (a * buoys[0].x)

        # create first waypoint
        wp1 = utils.get_midpoint(buoys[1], buoys[2])
        waypoints.append(utils.map_to_global(wp1[0], wp1[1]))

        # create second waypoint
        wp2_x = (math.sqrt((a ** 2 + 1)*(t ** 2) - k ** 2 + (2*a*j+2*b)*k -
                           ((a**2) * (j**2))-2*a*b*j - b**2) + (a*k) + j - a*b) / (a ** 2 + 1)
        wp2_z = (a * math.sqrt((a ** 2 + 1)*(t ** 2) - k ** 2 + (2*a*j+2*b)*k -
                               ((a**2) * (j**2))-2*a*b*j - b**2) + (k*(a**2)) + a * j + b) / (a ** 2 + 1)
        waypoints.append(utils.map_to_global(wp2_x, wp2_z))

        # create third waypoint
        wp3 = utils.get_extended_buoy(buoys[0], t=-3)
        waypoints.append(utils.map_to_global(wp3[0], wp3[1]))

        # create fourth waypoint
        wp4_x = -(math.sqrt((a ** 2 + 1)*(t ** 2) - k ** 2 + (2*a*j+2*b)*k -
                            ((a**2) * (j**2))-2*a*b*j - b**2) - (a*k) - j + a*b) / (a ** 2 + 1)
        wp4_z = -(a * math.sqrt((a ** 2 + 1)*(t ** 2) - k ** 2 + (2*a*j+2*b)*k -
                                ((a**2) * (j**2))-2*a*b*j - b**2) - (k*(a**2)) - a * j - b) / (a ** 2 + 1)
        waypoints.append(utils.map_to_global(wp4_x, wp4_z))

        # create fifth waypoint
        wp5 = utils.get_midpoint(buoys[1], buoys[2])
        waypoints.append(utils.map_to_global(wp5[0], wp5[1]))

        # create sixth waypoint
        wp6_x = SFR.ox
        wp6_z = SFR.oz
        waypoints.append(utils.map_to_global(wp6_x, wp6_z))
        rospy.loginfo("j: " + str(j))
        rospy.loginfo("k: " + str(k))
        rospy.loginfo("a: " + str(a))
        rospy.loginfo("b: " + str(b))

    else:
        # create first waypoint
        wp1_x = buoys[1].x - (buoy_radius + boat_radius + safety1)
        wp1_z = buoys[1].z
        waypoints.append(utils.map_to_global(wp1_x, wp1_z))

        # create second waypoint
        wp2_x = buoys[0].x + (buoy_radius + boat_radius + safety2)
        wp2_z = buoys[0].z
        waypoints.append(utils.map_to_global(wp2_x, wp2_z))

        # create third waypoint
        wp3_x = buoys[0].x
        wp3_z = buoys[0].z + buoy_radius + boat_radius + safety2
        waypoints.append(utils.map_to_global(wp3_x, wp3_z))

        # create fourth waypoint
        wp4_x = buoys[0].x - (buoy_radius + boat_radius + safety2)
        wp4_z = buoys[0].z
        waypoints.append(utils.map_to_global(wp4_x, wp4_z))

        # create fifth waypoint
        wp5_x = buoys[2].x + buoy_radius + boat_radius + safety1
        wp5_z = buoys[2].z
        waypoints.append(utils.map_to_global(wp5_x, wp5_z))

        # create sixth waypoint
        wp6_x = SFR.ox
        wp6_z = SFR.oz
        waypoints.append(utils.map_to_global(wp6_x, wp6_z))

    return waypoints


def execute():
    rospy.loginfo("attempting northern passage")
    rospy.loginfo("attempting to transition to northern passage")
    get_to_challenge()

    rospy.loginfo("attempting to pivot and indentify three buoys")
    pivot()

    waypoints = create_waypoints()
    sL, sR = pure_pursuit.execute(waypoints)
    thruster_utils.break_thrusters(sL, sR)
    for wp in waypoints:
        rospy.loginfo("determined waypoint: " + str(wp))

    finish("SUCCESS")
    rospy.loginfo("done northern passage")


def finish(outcome):
    rospy.loginfo(outcome)
    if outcome == "SUCCESS":
        # mark task as complete
        SFR.northern_passage_complete = True
        SFR.task = Task.DETERMINE_TASK
    else:
        # SFR.mode = Mode.REMOTE_CONTROL
        SFR.northern_passage_complete = True
        SFR.task = Task.DETERMINE_TASK
