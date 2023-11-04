"""
This file contains the software for completing the Beaching & Turtle Inspection
(previously Find A Seat) Task.
"""
from src.modes.tasks_enum import Task
import src.SFR as SFR
import tools.utils as utils
import src.path_execution.pure_pursuit as pure_pursuit
import numpy as np
from src.modes.movement_modes_enum import Mode
import rospy
import src.path_execution.thruster_utils as thruster_utils
import time


def findCorrectSign(objects, previously_seen=set()):
    # filter array to find correct sign (sign is complete side of dots)

    # filter array for correct sign

    signs = np.array(
        list(filter(lambda o: o not in previously_seen and o.label[0] == SFR.sign_color and o.label[3:] == "sign", objects)))
    s = previously_seen + signs

    if signs.size == 1:
        return signs, s
    else:
        return np.array([]), previously_seen


def findSigns(objects, previously_seen=set()):
    # filter array to find signs

    signs = np.array(
        list(filter(lambda o: o not in previously_seen and o.label[3:] == "sign", objects)))

    # sort objects by axis
    if len(signs) > 0:
        def get_attr(o): return getattr(o, 'x')
        get_axis_vals = np.vectorize(get_attr)
        signs = signs[get_axis_vals(signs).argsort()]

    s = previously_seen + signs

    if signs.size > 1:
        return signs, s
    else:
        return np.array([]), previously_seen


def pivot():
    # if correct sign not seen, pivot until seen
    correct_sign, s1 = findCorrectSign(SFR.objects)
    signs, s2 = findSigns(SFR.objects)

    start = time.time()
    sL = 1500
    sR = 1500
    # Boat takes 2 pi seconds to pivot in a full circle at 1 rad/sec
    if signs.size < 2 and correct_sign.size != 1:
        thruster_utils.sendValue(1550, 1450)
        sL, sR = 1550, 1450
        while signs.size < 2 and correct_sign.size != 1:
            if time.time() - start > 20:
                break
            signs, s2 = utils.filter_signs()
            correct_sign, s1 = utils.filter_correct_sign()
        thruster_utils.break_thrusters(sL, sR)

    # not enough signs seen
    if signs.size < 2 or correct_sign.size != 1:
        return 0, signs, correct_sign
    return 1, signs, correct_sign


def execute():
    # Determine assigned docking position from color/shape
    # Outline path to take
    # Follow path using PID control

    # Move forward for x seconds
    rospy.loginfo("attempting to complete beaching")
    result, signs, targetSign = pivot()
    if result == 0:
        finish("FAIL")
        rospy.loginfo("Signs not seen")
    else:
        for s in signs:
            rospy.loginfo("found sign: (" + str(s.x) + ", " + str(s.z) + ")")
        for s in targetSign:
            rospy.loginfo("found correct sign: (" +
                          str(s.x) + ", " + str(s.z) + ")")
        targetX = targetSign[0].x
        sideIndex = -1
        i = 0
        while i < signs.size:
            if signs[i].label[0] != SFR.sign_color:
                # x distance between sign at index i and target sign
                iDifference = np.absolute(signs[i].x - targetX)
                if sideIndex == -1:
                    sideIndex = i
                elif np.absolute(signs[sideIndex].x - targetX) > iDifference:
                    sideIndex = i
            i += 1
        sideSign = signs[sideIndex]

        rospy.loginfo("found sideSign: (" +
                      str(sideSign.x) + ", " + str(sideSign.z) + ")")
        waypoint = [utils.get_shifted_em(targetSign[0], sideSign, -1)]
        sL, sR = pure_pursuit.execute(waypoint)

        rospy.loginfo("determined waypoint: " + str(waypoint))
        rospy.loginfo("eggs found: " + targetSign[0].label[1])
        # move forward
        SFR.number_eggs = targetSign[0].label[1]
        thruster_utils.break_thrusters(sL, sR)

        start = time.time()
        sL, sR = thruster_utils.move_straight("B")
        while time.time() - start < 1:
            pass

        thruster_utils.break_thrusters(sL, sR)

        # move backward
        rospy.loginfo("done beaching")
        finish("SUCCESS")


def finish(result):
    rospy.loginfo(result)
    if result == "SUCCESS":
        SFR.beaching_complete = True
        SFR.task = Task.DETERMINE_TASK
    else:
        # SFR.task = Task.EXPLORE_REEF_END
        SFR.beaching_complete = True
        SFR.task = Task.DETERMINE_TASK
