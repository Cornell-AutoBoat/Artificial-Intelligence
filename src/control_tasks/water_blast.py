"""
This file contains the software for completing the Water_Blast Task
"""
from src.modes.tasks_enum import Task
import src.SFR as SFR
import numpy as np


def execute():
    # The boat must deliver enough water to raise the ball above the green line without
    # making contact with the dock the task sits on. The ASV may pump the water from the environment
    # versus storing it on board the vehicle
    if isComplete():
        SFR.WATER_BLAST = True
        SFR.task = Task.DETERMINE_TASK
    pass


def isComplete():
    pass


# TODO IMPORTANT: this code assumes that the shooter is run by servos that can simply be written an angle to.
# # # IF ANYONE FINDS OUT OTHERWISE LET CS TEAM KNOW. I will need to make this a control loop.


def FetchObjects():
    objects = np.array([])  # TODO: SUBSTITUTE ARRAY FOR ZED
    return objects


def filter(objects):
    # filter buoys array to contain 1 object: must target

    threshold = 0  # TODO: distance of buoys to pay attention to, 120ft?
    # buoys contains only green + red buoys
    buoys = objects[np.where(
        objects.label == "water_target")]  # TODO: CHANGE FOR CORRECT NAME

    # sort buoys buy z value
    close_buoys = np.sort(buoys[np.where(buoys.oz < threshold)], order='oz')

    # gets nearest 2 buoys
    if (len(close_buoys) == 1):
        return close_buoys[0:1]
    else:
        filter(FetchObjects())


def execute():
    # The boat must squirt

    orient(getXYZ())  # Big Bad Catamaran first orients its water sprayer
    # TODO: ece api start spraying + wait a certain amount of time
    while(False):  # TODO add timer
        orient(getXYZ())

    if isComplete():
        SFR.WATER_BLAST = True
        SFR.task = Task.DETERMINE_TASK


def getXYZ():
    bouy = filter(FetchObjects())[0]
    return [bouy.ox, bouy.oy, bouy.oz]


def orient(lst):
    hieght = heightRad(lst[0], lst[1], lst[2])
    orien = orienRad(lst[0], lst[1], lst[2])
    # move hieght servo #TODO: add correct ECE api functions
    # move width servo


def orienRad(x, y, z):  # equations here
    return np.arctan(y/x)  # BASIC NONSENSE CODE


def heightRad(x, y, z):  # equatinos here
    return np.arctan(y/z)  # BASIC NONSENSE CODe


def isComplete():
    # returns a boolean indicating whether the skeeball task is complete
    return True
