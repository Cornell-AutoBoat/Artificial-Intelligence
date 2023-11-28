"""
This file contains the software for completing the Beaching & Turtle Inspection
(previously Find A Seat) Task.
"""
from src.modes.tasks_enum import Task
import src.SFR as SFR
import src.tools.utils as utils
import numpy as np
from src.modes.movement_modes_enum import Mode
from src.tools import path_processing
import time
import math


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
    
    #old pivot code 
    '''if signs.size < 2 and correct_sign.size != 1:
        thruster_utils.sendValue(1550, 1450)
        sL, sR = 1550, 1450
        while signs.size < 2 and correct_sign.size != 1:
            if time.time() - start > 20:
                break
            signs, s2 = utils.filter_signs()
            correct_sign, s1 = utils.filter_correct_sign()
        thruster_utils.break_thrusters(sL, sR)  '''
    
    # If there are not enough signs seen and the correct sign is not seen --> pivot 
    # Once the correct number of signs is seen and the correct sign is found --> stop pivoting 

    if signs.size < 2 and correct_sign.size != 1:
        path_processing.send_to_controls("pivot_r")
        while signs.size < 2 and correct_sign.size != 1:
            if time.time() - start > 20:
                break
            signs, s2 = utils.filter_signs()
            correct_sign, s1 = utils.filter_correct_sign()
        path_processing.send_to_controls("stop")




    # not enough signs seen
    if signs.size < 2 or correct_sign.size != 1:
        return 0, signs, correct_sign
    return 1, signs, correct_sign

def pivotToGoalSign():
    # if correct sign not seen, pivot until seen
    correct_sign, s1 = findCorrectSign(SFR.objects)

    start = time.time()
    sL = 1500
    sR = 1500
    # Boat takes 2 pi seconds to pivot in a full circle at 1 rad/sec
    
    
    # If the correct sign is not seen --> pivot 
    # Once the the correct sign is found --> stop pivoting 

    if correct_sign.size != 1:
        path_processing.send_to_controls("pivot_r")
        while correct_sign.size != 1:
            if time.time() - start > 20:
                break
            signs, s2 = utils.filter_signs()
            correct_sign, s1 = utils.filter_correct_sign()
        path_processing.send_to_controls("stop")


    # not enough signs seen
    if correct_sign.size != 1:
        return 0, correct_sign
    return 1, correct_sign

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
        waypointOfTargetSign = targetSign[0]
        rospy.loginfo("determined waypoint: " + str(waypoint))
        

        ####### CHANGE (should perform same task as waypoint above, no longer needed)
        """"       
        rospy.loginfo("found sideSign: (" +
                     str(sideSign.x) + ", " + str(sideSign.z) + ")")
        yDifference = np.absolute(sideSign.y - targetY)
        xDifference = np.absolute(sideSign.x - targetX)
        slopeTarget = -1* yDifference / xDifference
        univertedSlope = xDifference / yDifference
        currLocX = SFR.tx
        currLocY = SFR.ty
        currLocZ = SFR.tz
        waypointX = ((univertedSlope * currLocX) +  currLocY + ((slopeTarget)*targetSign[0].x) - targetSign[0].y)/(slopeTarget - invertedSlope)
        waypointY = (slopeTarget(tx-targetSign[0].x)+ targetSign[0].y)
        waypoint2 = [utils.get_shifted_em(targetSign[0], sideSign, -1)]
        #sL, sR = pure_pursuit.execute(waypoint2) 
        """

        
        # Executes path to move in front of dock
        path = path_processing.process(waypoint)
        path_processing.send_to_controls("path", path)
        time.sleep(0.1)
        while not SFR.pp_done: 
            pass 
        path_processing.send_to_controls("stop")

        

        # ADD orientation code

        # pivot until you see the buoy 
        # get the potenital heading of the boat when it's facing the obejct  
        # pivot until the the boat's global heading is within the range of the object 

        num, correctSign = pivotToGoalSign()
        if num == 0: 
            finish("FAIL")
            rospy.loginfo("Signs not seen")
        elif num == 1: 
            localX = correctSign[0].x 
            localY = correctSign[0].y 
            # match the heading with the location of this sign 
            globalX, globalY = utils.map_to_global(localX, localY)
            psi = math.atan(globalY / globalX)
            # wrapping around to match the angle of the target sign to it's heading value
            headingVal = psi
            if globalX < 0 and globalY > 0 : 
                headingVal = -psi + (math.pi / 2)
            elif globalX < 0 and globalY < 0: 
                headingVal = - (math.pi - psi)
            elif globalX > 0 and globalY < 0: 
                headingVal = -psi
            
            # pivot until the heading matches the angle of the buoy with a 5 degree buffer 
            start = time.time()
            if (SFR.heading > headingVal + 0.0872665 or SFR.heading < headingVal - 0.0872665 ) : 
                path_processing.send_to_controls("pivot_l")
                while (SFR.heading >= headingVal + 0.0872665 or SFR.heading <= headingVal - 0.0872665 ):
                    if time.time() - start > 20:
                        break
                path_processing.send_to_controls("stop")

 

        # Move forward to pull into dock with the target sign as waypoint
        path2 = path_processing.process(waypointOfTargetSign)
        path_processing.send_to_controls("path", path2)
        time.sleep(0.1)
        while not SFR.pp_done: 
            pass 
        path_processing.send_to_controls("stop")

        # Time the boat stays parked in the dock
        time.sleep(5) 

        # Move backward to back out of dock
        path_processing.send_to_controls("bwd")
        #Note: the time to back out of the dock can be changed depending on how far it moves
        time.sleep(3)
        path_processing.send_to_controls("stop")


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
