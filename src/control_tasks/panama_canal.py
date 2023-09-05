"""
This file contains the software for completing the Panama Canal (previously 
Mandatory Navigation Channel) Task.
"""
import src.SFR as SFR
import numpy as np
import src.control_tasks.utils as utils
import src.path_execution.pure_pursuit as pure_pursuit
import src.path_execution.thruster_utils as thruster_utils
import src.path_execution.piecewise as piecewise
from src.modes.tasks_enum import Task
import time
import rospy

# Panama Canal:

# UR  x  UG
#
#
# LR  x  LG

def backup():
    w = [[SFR.tx, SFR.tz + 4.5]]
    # sL, sR = pure_pursuit.execute(w, lookahead=2.0)
    piecewise.follow(w)
    thruster_utils.break_thrusters(sL, sR)

def pivot(seen):
    # [[close red, close green], [far red, far green]]
    gates = np.array([[None, None], [None, None]])

    # Search left then search right. Conditions: a gate buoy has not yet been
    # identified and we haven't turned 90 degrees (pi/4 radians)
    start = time.time()
    sL, sR = 1450, 1550
    thruster_utils.sendValue(sL, sR)
    while (not gates[0, 0] or not gates[0, 1]):
        if time.time() - start > 4:
            break
        buoys, seen = utils.filter_objects(
            ["green-column-buoy", "red-column-buoy"], seen)
        for b in buoys:
            if b.z < 4.0:   # buoy is less than 4 meters away
                if b.label == "red-column-buoy":
                    gates[0, 0] = b
                else:
                    gates[0, 1] = b
            else:
                if b.label == "red-column-buoy":
                    gates[1, 0] = b
                else:
                    gates[1, 1] = b
    thruster_utils.break_thrusters(sL, sR)

    if not gates[0, 0] or not gates[0, 1]:
        start = time.time()
        sL, sR = 1550, 1450
        thruster_utils.sendValue(sL, sR)
        while (not gates[0, 0] or not gates[0, 1]):
            if time.time() - start > 8:
                break
            buoys, seen = utils.filter_objects(
                ["green-column-buoy", "red-column-buoy"], seen)
            for b in buoys:
                if b.z < 4.0:   # buoy is less than 4 meters away
                    if b.label == "red-column-buoy":
                        gates[0, 0] = b
                    else:
                        gates[0, 1] = b
                else:
                    if b.label == "red-column-buoy":
                        gates[1, 0] = b
                    else:
                        gates[1, 1] = b
        thruster_utils.break_thrusters(sL, sR)

    if not gates[0, 0] or not gates[0, 1]:
        start = time.time()
        sL, sR = 1450, 1550
        thruster_utils.sendValue(sL, sR)
        while (not gates[0, 0] or not gates[0, 1]):
            if time.time() - start > 4:
                break
            buoys, seen = utils.filter_objects(
                ["green-column-buoy", "red-column-buoy"], seen)
            for b in buoys:
                if b.z < 4.0:   # buoy is less than 4 meters away
                    if b.label == "red-column-buoy":
                        gates[0, 0] = b
                    else:
                        gates[0, 1] = b
                else:
                    if b.label == "red-column-buoy":
                        gates[1, 0] = b
                    else:
                        gates[1, 1] = b
        thruster_utils.break_thrusters(sL, sR)

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
        mid_x1, mid_z1 = utils.get_extended_midpoint(buoys[0], buoys[1], t=1)
        mid_x2, mid_z2 = utils.get_extended_midpoint(buoys[2], buoys[3], t=3)
        waypoints.append(utils.map_to_global(mid_x1, mid_z1))
        waypoints.append(utils.map_to_global(mid_x2, mid_z2))
        sL, sR = pure_pursuit.execute(waypoints)
        time.sleep(1)
        thruster_utils.break_thrusters(sL, sR)
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
                        mid_x, mid_z = utils.get_extended_midpoint(b1, b2)
                        waypoints.append(utils.map_to_global(mid_x, mid_z))
                        gates_passed += 1
                sL, sR = pure_pursuit.execute(waypoints)
        thruster_utils.break_thrusters(sL, sR)
        finish("SUCCESS")


def finish(outcome):
    rospy.loginfo(outcome)
    if outcome == "SUCCESS":
        # mark task as complete
        SFR.panama_canal_complete = True
        SFR.task = Task.DETERMINE_TASK
        SFR.done = True
    else:
        # SFR.task = Task.EXPLORE_REEF_END
        SFR.done = True