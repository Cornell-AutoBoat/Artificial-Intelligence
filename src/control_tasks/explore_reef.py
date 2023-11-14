"""
This file contains the software for completing the Explore the Coral Reef 
(previously Return Home) Task.
"""
from src.modes.tasks_enum import Task
import numpy as np
from src.tools.utils import filter_objects, get_midpoint, map_to_global
import src.SFR as SFR
from tools.a_star import get_waypoints
from src.tools.path_processing import send_to_controls, process
from src.path_execution import pure_pursuit
from src.path_execution import thruster_utils
import time


def pivot_360():
     # turn 360 degrees and return the midpoint of the function
     midpoint = None
     # pivot right some amount more than 0.5 medians
     start_time = time.time()
     buoys, seen = filter_objects(["black_buoy"])
    
     black_buoys
     while (not buoys):
         if time.time() - start_time>4:
             break 
         buoys,seen = filter_objects(["black_buoy"], seen)
         black_buoys= buoys
    
     send_to_controls("stop")
     
    #  if black_buoys.length <2:
    #      return None 
     
     midpoint = get_midpoint(black_buoys[0], black_buoys[1])
     global_midpoint = map_to_global(midpoint[0], midpoint[1])
     return global_midpoint

def start():
     # Identify gate of black buoys 
     # Store midpoint to SFR
     SFR.explore_reef_location = pivot_360()  # pivot 360 degrees
     SFR.task = Task.DETERMINE_TASK

def execute_end(repeat=True):
     # Use A* to navigate through gate
     # Get the current coordinates

     #retrieve global coordinates of yellow buoy of speed challenge
     #A* and pure pursuit to black buoys below
     y_buoy_x= yellow_buoy.tx
     y_buoy_y=yellow_buoy.ty-50
     waypoint_explore=[]
     waypoint_explore.append([y_buoy_x,y_buoy_y])
     path = process(waypoint_explore)
     send_to_controls("path", path)
     time.sleep(0.1)
     while not SFR.pp_done:
         pass 
     send_to_controls("stop")

     # sL, sR = pure_pursuit.execute(waypoint_explore, sec=3)

     #go to yellow buoy
     #A* and pure pursuit to black buoys below


 #insert cases here 
 #insert cases here 
     goal = SFR.explore_reef_location
     objects = SFR.objects
     objects_map = [[0]*100 for _ in range(100)]

     for object in objects:
         global_coords = map_to_global(object.x, object.y)
         object_x, object_y = global_coords[0], global_coords[1]
         object_row_index = object_x - SFR.tx + 50
         object_col_index = object_y - SFR.ty + 50
         if object_row_index < 100 and object_col_index < 100:
             objects_map[int(object_row_index)][int(object_col_index)] = 1

     # TODO: consider finding better end point
     goal_x = np.clip(int(goal[0] - SFR.tx + 50), 0, 99)
     goal_y = np.clip(int(goal[1] - SFR.ty + 50), 0, 99)
     end = (goal_x, goal_y)
     waypoint_indices = get_waypoints(objects_map, (50, 50), end)
     waypoint_global = []

     for index_x, index_y in waypoint_indices:
         global_x = SFR.tx + index_x - 50
         global_y = SFR.ty + index_y - 50

         waypoint_global.append([global_x, global_y])


     path = process(waypoint_global)
     send_to_controls("path", path)
     time.sleep(0.1)
     while not SFR.pp_done: 
         pass 
     send_to_controls("stop")

     # sL, sR = pure_pursuit.execute(waypoint_global, sec=3)
     # thruster_utils.break_thrusters(sL, sR)

     if isComplete() or not(repeat):
         SFR.explore_reef_complete = True
         SFR.task = Task.DETERMINE_TASK
     else:
        execute_end()
        # add sleep
def isComplete():
     # returns a boolean indicating whether the explore the coral reef task is complete
     goal = SFR.explore_reef_location
     if abs(goal[0] - SFR.tx) < 2 and abs(goal[1] - SFR.ty) < 2:
         return True
     return False