#!/usr/bin/env python3
"""
Main control sequence. Determines movement mode, then loops until competition 
course has been completed.
"""

from src import autonomous_control, SFR
from src.modes.movement_modes_enum import Mode
import rospy
import numpy as np
import time

from test.msg import ZEDdata, State, SensorReadings, MotionPlans, Done


def main_control_loop():
    rospy.loginfo("entering main control loop")
    # stall while the system is off or remote control is on
    print(SFR.system_on)
    print(SFR.RC_on)
    while (not SFR.system_on) or SFR.RC_on:
        pass

    # Once mode autonomous, call autonomous execute
    autonomous_control.execute()
    # print("sending lin")
    # basic_tasks.test_send_signals_lin()


def callback_sensors(msg):
    SFR.tx = msg.pos_x
    SFR.ty = msg.pos_y
    SFR.heading = msg.heading


def callback_zed(msg):
    SFR.objects = np.array(msg.objects)
    # print(SFR.objects)


def callback_state(msg):
    SFR.alive = msg.alive
    SFR.autonomous = msg.autonomous


def callback_done(msg):
    SFR.pp_done = msg.done


if __name__ == "__main__":
    rospy.init_node('AINode', anonymous=True)
    rospy.Subscriber('zed-data', ZEDdata, callback_zed)
    rospy.Subscriber('alive_auto', State, callback_state)
    rospy.Subscriber('sensors', SensorReadings, callback_sensors)
    rospy.Subscriber('done', Done, callback_done)
    pub = rospy.Publisher('motion-plans', MotionPlans, queue_size=10)
    SFR.mcPub = pub
    time.sleep(3)
    while not SFR.execution_done and not rospy.is_shutdown():
        main_control_loop()

    rospy.loginfo(
        "\\\\\\\\\\\\\\\\\\\\\\\\\\finished main control loop\\\\\\\\\\\\\\\\\\\\\\\\\\")

