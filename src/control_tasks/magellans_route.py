from src.tools.utils import filter_objects, get_midpoint, map_to_global, get_extended_midpoint
import src.SFR as SFR
from src.tools.a_star import get_waypoints
import numpy as np
from src.modes.tasks_enum import Task
#from src.path_execution import pure_pursuit
from tools.a_star import get_waypoints
#from src.path_execution import thruster_utils
from tools.path_processing import send_to_controls, process
import rospy
from src.buoys import Buoy
import time

def print_array(array):
    print('\n'.join([str(row)[80:200] for row in array]))

def main():
    """
    Executes main functions for the task
    """
    transition_to_task()
    execute_task()

def transition_to_task():
    """
    Finds the first gate to begin the task, which is the first red and green 
    buoy, and calculates the midpoint between the buoys
    Executes the path to the midpoint using pure pursuit
    """
    rospy.loginfo("finding first gate")
    begin_buoys, _ = filter_objects(["red-buoy", "green-buoy"])
    start_time = time.time()
    while begin_buoys.size < 2 and time.time() - start_time < 5:
        begin_buoys, _ = filter_objects(["red-buoy", "green-buoy"])
    if begin_buoys.size < 2:
        midpoint = [0, 3]
        rospy.loginfo("could not find gate, setting gate as straight forward")
    else:
        midpoint = get_midpoint(begin_buoys[0], begin_buoys[1])
    global_mid = map_to_global(midpoint[0], midpoint[1])
    path = process([global_mid])
    rospy.loginfo(f"found first gate at {global_mid}")
    send_to_controls("path", path)
    rospy.loginfo("moving towards first gate")

    time.sleep(0.1)
    while not SFR.pp_done:
        pass
    send_to_controls("stop")
    #sL, sR = pure_pursuit.execute([global_mid])
    #thruster_utils.break_thrusters(sL, sR)

def execute_task(repeat = True):
    if not SFR.system_on:
        rospy.loginfo("kill switch activated, quitting magellans route")
        # Consider marking task as done
        return
    
    objects_map = [[0]*100 for _ in range(100)]
    object_count = 0

    rospy.loginfo("finding buoys")
    objects, _ = filter_objects(["red-buoy"])
    objects = np.array(list(filter(lambda o: o.x > -5 and o.x < 5, objects)))
    objects = np.insert(objects, 0, Buoy(label="red-buoy", x = -2, y = -4, z = 0)) 
    #inserts at beginning of 'objects' so that the boat doesn't go backwards
    add_objects_to_map(objects, objects_map, fill_between=True)
    object_count += len(objects) - 1
    last_red = objects[-1]
    objects, _ = filter_objects(["green-buoy"])
    objects = np.array(list(filter(lambda o: o.x > -5 and o.x < 5, objects)))
    objects = np.insert(objects, 0, Buoy(label="green-buoy", x = 2, y = -4, z = 0))
    add_objects_to_map(objects, objects_map, fill_between=True)
    object_count += len(objects) - 1
    last_green = objects[-1]
    objects, _ = filter_objects(["yellow-buoy"])
    objects = np.array(list(filter(lambda o: o.x > -5 and o.x < 5, objects)))
    add_objects_to_map(objects, objects_map, fill_between=False)
    nummber_eggs += len(objects) # change this global variable if necessary
    
    objects, _ = filter_objects(["black-buoy"])
    objects = np.array(list(filter(lambda o: o.x > -5 and o.x < 5, objects)))
    add_objects_to_map(objects, objects_map, fill_between=False)

    # print_array(objects_map)

    rospy.loginfo("getting goal point")
    # WARNING: DOES NOT WORK WITH SAME Z COORDS, DIVIDE BY ZERO
    goal = get_extended_midpoint(last_red, last_green)
    goal = map_to_global(goal[0], goal[1])
    rospy.loginfo(f"goal point {goal}")

    goal_x = np.clip(int((goal[0] - SFR.tx)*2 + 50), 0, 99)
    goal_y = np.clip(int((goal[1] - SFR.ty)*2 + 50), 0, 99)
    # these two lines are transforming the goal coordinates from local to global
    end = (goal_x, goal_y)
    objects_map[goal_x][goal_y] = 0
    waypoint_indices = get_waypoints(objects_map, (50, 50), end, allow_diagonal_movement=True) #what is (50,50)
    # gets all the waypoints for the path form objects_map using A star
    if waypoint_indices == None:
        rospy.loginfo("can not find waypoints")
        SFR.magellans_route_complete = True
        SFR.task = Task.DETERMINE_TASK
    waypoint_global = []

    rospy.loginfo("finding global waypoints, performing A star")
    for index_x, index_y in waypoint_indices:
        # appends global coordinates for all waypoints from waypoint_indices
        global_x = SFR.tx + (index_x - 50)/2
        global_y = SFR.ty + (index_y - 50)/2

        waypoint_global.append([global_x, global_y])

    path = process(waypoint_global)
    send_to_controls("path", path)
    
    rospy.loginfo(f"waypoints {waypoint_global}")
    rospy.loginfo("executing path")
    
    time.sleep(0.1)
    while not SFR.pp_done:
        pass
    send_to_controls("stop")
    #sL, sR = pure_pursuit.execute(waypoint_global, sec = 3, lookahead = 0.5)
    # execute using pure pursuit

    # FINISH
    if object_count <= 2:
        rospy.loginfo("finished due to no more buoys in proximity")
        send_to_controls("stop")
        #thruster_utils.break_thrusters(sL, sR)
        SFR.magellans_route_complete = True
        SFR.task = Task.DETERMINE_TASK
        return

    if repeat:
        execute_task()
        # TODO: add sleep

def add_objects_to_map(objects, objects_map, fill_between = False):
    """
    This function is designed to add objects to a 2D map and optionally fill 
    the space between objects on the map.
    """
    object_indices = []
    prev = []
    for i in range(len(objects)):
        # print(objects[i].x, objects[i].z)
        global_coords = map_to_global(objects[i].x, objects[i].y)
        object_x, object_y = global_coords[0], global_coords[1]
        object_row_index = (object_x - SFR.tx)*2 + 50
        object_col_index = (object_y - SFR.ty)*2 + 50
        # print(object_row_index, object_col_index)
        if object_row_index < 98 and object_col_index < 98 and object_row_index > 1 and object_col_index > 1:
            object_indices.append((object_row_index, object_col_index))
            if fill_between and i > 0:
                fill(object_indices, prev[0], prev[1], object_row_index, object_col_index)
            prev = [object_row_index, object_col_index]

    for object_row_index, object_col_index in object_indices:
        if (fill_between or objects_map[int(object_row_index) - 1][int(object_col_index)] == 0 
             and objects_map[int(object_row_index) - 1][int(object_col_index) - 1] == 0 
             and objects_map[int(object_row_index) - 1][int(object_col_index) + 1] == 0 
             and objects_map[int(object_row_index) + 1][int(object_col_index)] == 0 
             and objects_map[int(object_row_index) + 1][int(object_col_index) - 1] == 0 
             and objects_map[int(object_row_index) + 1][int(object_col_index) + 1] == 0):
                objects_map[int(object_row_index)][int(object_col_index)] = 1

def fill(indices, x1, y1, x2, y2):
    x_inc = 1
    if x2 < x1:
        x_inc = -1

    y_inc = 1
    if y2 < y1:
        y_inc = -1

    x_i = 0
    y_i = 0
    while x_i + x1 != x2 and y_i + y1 != y2:
        indices.append((x1 + x_i, y1 + y_i))
        indices.append((x1 + x_i + x_inc, y1 + y_i))
        indices.append((x1 + x_i, y1 + y_i + y_inc))
        x_i += x_inc
        y_i += y_inc

    while x_i + x1 != x2:
        indices.append((x1 + x_i, y2))
        x_i += x_inc

    while y_i + y1 != y2:
        indices.append((x2, y1 + y_i))
        y_i += y_inc
