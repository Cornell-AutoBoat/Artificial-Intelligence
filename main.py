#!/usr/bin/env python3
"""
Main control sequence. Determines movement mode, then loops until competition 
course has been completed.
"""

from src import autonomous_control
from src import SFR
from src.modes.movement_modes_enum import Mode
import rospy
import numpy as np
from src.control_tasks import utils
from src.path_execution import thruster_utils
from src.path_execution import pid
import time
from src.control_tasks import basic_tasks

from test.msg import ZEDdata, state, MCdata
# from test.msg import state
# from test.msg import MCdata


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


def callback_zed(msg):
    SFR.tx = msg.tx
    SFR.ty = msg.ty
    SFR.tz = msg.tz
    # print("x: " + str(SFR.tx))
    # print("y: " + str(SFR.ty))
    # print("z: " + str(SFR.tz))
    SFR.ox = msg.ox
    SFR.oy = msg.oy
    SFR.oz = msg.oz
    SFR.ow = msg.ow
    currHeading = -utils.get_yaw() + np.pi/2
    if currHeading < 0:
        currHeading += 2*np.pi
    # print("heading: " + str(currHeading))
    # print(utils.map_to_global(2, 3))
    SFR.lin_v += msg.lin_a*(time.time() - SFR.prev_time)
    SFR.prev_time = time.time()
    SFR.ang_vx = msg.ang_vx
    SFR.ang_vy = msg.ang_vy
    SFR.ang_vz = msg.ang_vz
    SFR.objects = np.array(msg.objects)
    # print(SFR.objects)

def callback_electrical(msg):
    SFR.system_on = msg.alive
    SFR.RC_on = not msg.autonomous

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', ZEDdata, callback_zed)
    rospy.Subscriber('electrical', state, callback_electrical)
    pub = rospy.Publisher('MCinfo', MCdata, queue_size=10)
    SFR.mcPub = pub
    time.sleep(3)
    while not SFR.done and not rospy.is_shutdown():
        main_control_loop()

    rospy.loginfo(
        "\\\\\\\\\\\\\\\\\\\\\\\\\\finished main control loop\\\\\\\\\\\\\\\\\\\\\\\\\\")

    # thruster_utils.kill_system()
