"""
This file contains the software for completing the Panama Canal (previously 
Mandatory Navigation Channel) Task.
"""
import src.SFR as SFR
import numpy as np
from src.tools import utils
from src.tools.path_processing import send_to_controls, process
from src.modes.tasks_enum import Task
import time
import rospy

# Panama Canal:

# UR  x  UG
#
#
# LR  x  LG


def pivot(seen):
    # [[close red, close green], [far red, far green]]
    gates = np.array([[None, None], [None, None]])

    # Search left then search right. Conditions: a gate buoy has not yet been
    # identified and we haven't turned 90 degrees (pi/4 radians)
    start = time.time()
    send_to_controls("pivot_l")
    while (not gates[0, 0] or not gates[0, 1]):
        if time.time() - start > 4:
            break
        buoys, seen = utils.filter_objects(
            ["green-column-buoy", "red-column-buoy"], seen)
        for b in buoys:
            if b.y < 4.0:   # buoy is less than 4 meters away
                if b.label == "red-column-buoy":
                    gates[0, 0] = b
                else:
                    gates[0, 1] = b
            else:
                if b.label == "red-column-buoy":
                    gates[1, 0] = b
                else:
                    gates[1, 1] = b
    send_to_controls("stop")

    if not gates[0, 0] or not gates[0, 1]:
        start = time.time()
        send_to_controls("pivot_r")
        while (not gates[0, 0] or not gates[0, 1]):
            if time.time() - start > 8:
                break
            buoys, seen = utils.filter_objects(
                ["green-column-buoy", "red-column-buoy"], seen)
            for b in buoys:
                if b.y < 4.0:   # buoy is less than 4 meters away
                    if b.label == "red-column-buoy":
                        gates[0, 0] = b
                    else:
                        gates[0, 1] = b
                else:
                    if b.label == "red-column-buoy":
                        gates[1, 0] = b
                    else:
                        gates[1, 1] = b
        send_to_controls("stop")

    if not gates[0, 0] or not gates[0, 1]:
        start = time.time()
        send_to_controls("pivot_l")
        while (not gates[0, 0] or not gates[0, 1]):
            if time.time() - start > 4:
                break
            buoys, seen = utils.filter_objects(
                ["green-column-buoy", "red-column-buoy"], seen)
            for b in buoys:
                if b.y < 4.0:   # buoy is less than 4 meters away
                    if b.label == "red-column-buoy":
                        gates[0, 0] = b
                    else:
                        gates[0, 1] = b
                else:
                    if b.label == "red-column-buoy":
                        gates[1, 0] = b
                    else:
                        gates[1, 1] = b
        send_to_controls("pivot_r")

    return gates, seen


def execute():
    # Identify a gate
    # Calculate midpoint of gate as a waypoint
    # Send waypoints to pure pursuit
    # Make adjustments as needed

    rospy.loginfo("attempting panama canal")

    buoys, _ = utils.filter_objects(["red-column-buoy", "green-column-buoy"])
    waypoints = []
    if len(buoys) == 4:
        rospy.loginfo("all buoys seen immediately")
        mid_x1, mid_y1 = utils.get_extended_midpoint(buoys[0], buoys[1], t=1)
        mid_x2, mid_y2 = utils.get_extended_midpoint(buoys[2], buoys[3], t=3)
        waypoints.append(utils.map_to_global(mid_x1, mid_y1))
        waypoints.append(utils.map_to_global(mid_x2, mid_y2))

        path = process(waypoints)
        send_to_controls("path", path)
        while not SFR.pp_done:
            pass
        send_to_controls("stop")
        finish("SUCCESS")
    else:
        rospy.loginfo("can't see all buoys")
        seen = set()
        gates_passed = 0
        # completion criteria: 2 gates identified and passed through
        while gates_passed < 2:
            gates, seen = pivot(seen)
            if not gates[0, 0] or not gates[0, 1]:
                finish("FAIL")
                return
            else:
                waypoints = []
                for gate in gates:
                    b1, b2 = gate[0], gate[1]
                    if b1 and b2:
                        # calculate the midpoint in local coords
                        mid_x, mid_y = utils.get_extended_midpoint(b1, b2)
                        waypoints.append(utils.map_to_global(mid_x, mid_y))
                        gates_passed += 1
                path = process(waypoints)
                send_to_controls("path", path)
                while not SFR.pp_done:
                    pass
        send_to_controls("stop")
        finish("SUCCESS")


def finish(outcome):
    rospy.loginfo(outcome)
    if outcome == "SUCCESS":
        # mark task as complete
        SFR.panama_canal_complete = True
        SFR.task = Task.DETERMINE_TASK
        SFR.execution_done = True
    else:
        # SFR.task = Task.EXPLORE_REEF_END
        SFR.execution_done = True
