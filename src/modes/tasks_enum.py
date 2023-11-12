"""
Enumerates the different tasks for the boat.
"""
from enum import Enum


class Task(Enum):
    DETERMINE_TASK = 0

    # Objectives
    EXPLORE_REEF_START = 1
    PANAMA_CANAL = 2
    MAGELLANS_ROUTE = 3
    NORTHERN_PASSAGE = 4
    BEACHING = 5
    EXPLORE_REEF_END = 6
    BASIC_TASKS = 9
    PID_TUNE = 10
    TASKS_VISUALIZATION = 11
