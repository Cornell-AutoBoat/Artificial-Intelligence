"""
This file contains the software for completing the Explore the Coral Reef 
(previously Return Home) Task.
"""
from src.modes.tasks_enum import Task
import numpy as np
from tools.utils import get_yaw, filter_objects, get_midpoint, map_to_global
import src.SFR as SFR
from tools.a_star import get_waypoints
from src.path_execution import pure_pursuit
from src.path_execution import thruster_utils
import time


def pivot_360():
    # turn 360 degrees and return the midpoint of the function
    midpoint = None
    found = False
    orig_yaw = get_yaw()
    # pivot right some amount more than 0.5 medians
    start_time = time.time()
    thruster_utils.sendValue(1450, 1550)
    sL, sR = 1450, 1550
    while time.time() - start_time < 10 and not(found):
        thruster_utils.break_thrusters(sL, sR)
        if not(found):
            black_objects = filter_objects(["black-buoy"])
            if len(black_objects) == 2:
                mid_loc = get_midpoint(black_objects[0], black_objects[1])
                midpoint = map_to_global(mid_loc[0], mid_loc[1])
                found = True
    thruster_utils.break_thrusters(sL, sR)
    thruster_utils.sendValue(1550, 1450)
    sL, sR = 1550, 1450
    while np.abs(orig_yaw - get_yaw()) > 0.5:
        pass
    thruster_utils.break_thrusters(sL, sR)
    return midpoint


def start():
    # Identify gate of black buoys
    # Store midpoint to SFR
    SFR.explore_reef_location = pivot_360()  # pivot 360 degrees
    SFR.task = Task.DETERMINE_TASK


def execute_end(repeat=True):
    # Use A* to navigate through gate
    # Get the current coordinates

    goal = SFR.explore_reef_location
    objects = SFR.objects
    objects_map = [[0]*100 for _ in range(100)]

    for object in objects:
        global_coords = map_to_global(object.x, object.z)
        object_x, object_z = global_coords[0], global_coords[1]
        object_row_index = object_x - SFR.tx + 50
        object_col_index = object_z - SFR.tz + 50
        if object_row_index < 100 and object_col_index < 100:
            objects_map[int(object_row_index)][int(object_col_index)] = 1

    # TODO: consider finding better end point
    goal_x = np.clip(int(goal[0] - SFR.tx + 50), 0, 99)
    goal_z = np.clip(int(goal[1] - SFR.tz + 50), 0, 99)
    end = (goal_x, goal_z)
    waypoint_indices = get_waypoints(objects_map, (50, 50), end)
    waypoint_global = []

    for index_x, index_z in waypoint_indices:
        global_x = SFR.tx + index_x - 50
        global_z = SFR.tz + index_z - 50

        waypoint_global.append([global_x, global_z])

    sL, sR = pure_pursuit.execute(waypoint_global, sec=3)
    thruster_utils.break_thrusters(sL, sR)

    if isComplete() or not(repeat):
        SFR.explore_reef_complete = True
        SFR.task = Task.DETERMINE_TASK
    else:
        execute_end()
        # add sleep


def isComplete():
    # returns a boolean indicating whether the explore the coral reef task is complete
    goal = SFR.explore_reef_location
    if abs(goal[0] - SFR.tx) < 2 and abs(goal[1] - SFR.tz) < 2:
        return True
    return False
