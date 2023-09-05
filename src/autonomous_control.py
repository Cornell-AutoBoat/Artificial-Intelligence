"""
This file determines the exact autonomous execution sequence.
"""
import logging
from src.modes.tasks_enum import Task
import src.SFR as SFR
import src.control_tasks.panama_canal as panama_canal
import src.control_tasks.magellans_route as magellans_route
import src.control_tasks.northern_passage as northern_passage
import src.control_tasks.beaching as beaching
import src.control_tasks.skeeball as skeeball
import src.control_tasks.water_blast as water_blast
import src.control_tasks.explore_reef as explore_reef
import src.control_tasks.basic_tasks as basic_tasks
import src.path_execution.pid as pid


def determine_task():
    # Assume a strict order of task completion. Order corresponds to task enumerations.
    # Update the SFR so that the next loop calls the appropriate transition function.
    # If the last task (explore reef) has been completed, exit out of main control loop

    if SFR.explore_reef_complete:
        SFR.done = True
    else:
        SFR.task = Task(SFR.task.value + 1).name


def execute():
    logging.info("autonomous control execute")
    if SFR.task == Task.BASIC_TASKS:
        basic_tasks.execute()
    elif SFR.task == Task.PID_TUNE:
        pid.pid_control(0, 1)
    elif SFR.task == Task.DETERMINE_TASK:
        determine_task()
    elif SFR.task == Task.EXPLORE_REEF_START:
        explore_reef.start()
    elif SFR.task == Task.PANAMA_CANAL:
        panama_canal.execute()
    elif SFR.task == Task.MAGELLANS_ROUTE:
        magellans_route.main()
    elif SFR.task == Task.NORTHERN_PASSAGE:
        northern_passage.execute()
    elif SFR.task == Task.BEACHING:
        beaching.execute()
    else:
        explore_reef.execute_end()
