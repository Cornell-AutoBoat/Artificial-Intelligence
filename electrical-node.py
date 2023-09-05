#!/usr/bin/env python3
import rospy 
from test.msg import state, MCdata
from src import SFR
import serial
# import time
##  state has two bools: autonomous, alive
# change to jetson port
try:
    serialCOM = serial.Serial(port='/dev/ttyACM1', baudrate=115200, timeout=1) # Set up serial communication
except: 
    serialCOM = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1) # Set up serial communication


global_alive = True
global_autonomous = False

def callback_MC(msg):
    print("called", msg.sR)
    global global_autonomous
    global serialCOM
    if (global_autonomous): 
        #  autonomous #print("in auto")
        # Update the values in SFR
        mc = str((f"MC R{str(msg.sR)} L{str(msg.sL)}\n"))
        print(mc)
        serialCOM.write(mc.encode())
        # old_sl = SFR.sL
        # old_sR = SFR.sR

        if (msg.kill):
            kill_switch = "KILL\n"
            # print(kill_switch)
            serialCOM.write(kill_switch.encode())
# import SFR # need to change

# Store the old SFR.rl and SFR.sl values in global variables

def main():
  rospy.init_node('electricalsignal', anonymous=True)
  pub = rospy.Publisher('electrical', state, queue_size=10)
  rospy.Subscriber('MCinfo', MCdata,  callback_MC)
  rate = rospy.Rate(10) # 10hz

#   old_sl = SFR.sL
#   old_sR = SFR.sR

  global global_autonomous
  global global_alive 
  global serialCOM


  while not rospy.is_shutdown():
      # hello_str = "hello world %s" % rospy.get_time()
      # rospy.loginfo(hello_str)
      # read from sfr?
      rate.sleep()
      msg = state()
      if (serialCOM.inWaiting() > 0):
          data = serialCOM.readline().decode().strip()
        #   print(data)

          if data == "KILL":
              global_alive = False
            #   print("KILL")
          elif data == "UNKILL":
            #   print("UNKILL")
              global_alive = True
          elif data == "MAN":
            #   print("MAN")
              global_autonomous = False
          elif data == "AUTO":
              print("AUTO")
              global_autonomous = True
      msg.autonomous = global_autonomous
      msg.alive = global_alive
      pub.publish(msg)

      
  serialCOM.close()

if __name__ == "__main__":
    main()
    	





      