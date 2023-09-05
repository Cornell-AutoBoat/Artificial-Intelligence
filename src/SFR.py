"""
A file of global variables describing the state of the boat.
"""

import numpy as np
from src.modes.movement_modes_enum import Mode
from src.modes.tasks_enum import Task

# Boat competition state variables.
mode = Mode.REMOTE_CONTROL
task = Task.BASIC_TASKS
done = False

# system_on and RS_on are written by the serial node

# Indictates if the remote kill switch has been activated. If False, it has been
# activated and system is off (i.e. no power will be sent to motors)
system_on = True

# Indicates if the RC knob on the remote controller has been turned on. If True,
# our code should stall. If False, autonomous mode is activated and our code will run
RC_on = True

# Values to be sent to the left and right thrusters
sL = 1500
sR = 1500

# Indicates whether the MC code wants to kill the system
kill = False

# Boat position variables. Updated by the ZED node.
tx, ty, tz = 0.0, 0.0, 0.0
ox, oy, oz, ow = 0.0, 0.0, 0.0, 0.0

# Velocities. Updated by the ZED node.
prev_time = 0
lin_v = 0
ang_vx = 0
ang_vy = 0
ang_vz = 0

# List of Buoy type objects that the boat sees and their positionality data.
# Updated by the ZED node.
objects = np.array([])

# Task tracking. True = task has been completed.
panama_canal_complete = False
magellans_route_complete = False
northern_passage_complete = False
beaching_complete = False
water_blast_complete = False
skeeball_complete = False
explore_reef_complete = False

# Global location of the midpoint of the coral reef gate
explore_reef_location = (0, 0)

# Sign color for beaching task and number of eggs reported
sign_color = "B"
number_eggs = 0

# Motor control publisher. Publishes motor and kill signals to electrical node.
mcPub = None