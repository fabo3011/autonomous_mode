#!/usr/bin/env python

#node by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

#Project ALVIN (Autonomous Logic Virtual Intelligence n' Navigation)

import rospy
import math
from nav2d_operator.msg import cmd
from std_msgs.msg import Float32


#constant limit sets
#maxVel = 1.0
#maxTurn = 0.6

#base controller constants
width_robot =  1.5

#global variables
cmdVel = cmd()

#global flag
dataChanged = False

def cmdCb(data):
    global cmdVel
    global dataChanged
    
    cmdVel.Velocity = data.Velocity
    cmdVel.Turn     = data.Turn
    dataChanged = True


def BaseController():

    global cmdVel
    global dataChanged

    pub = rospy.Publisher('cmd', cmd, queue_size=10)

    leftVelPub  = rospy.Publisher('/BaseController/left_vel',  Float32, queue_size=10)
    rightVelPub = rospy.Publisher('/BaseController/right_vel', Float32, queue_size=10)
    rospy.init_node('BaseController', anonymous=True)
    rospy.Subscriber('cmd',cmd,cmdCb)


    r = rospy.Rate(5) #5 Hz (rate of /gpstarget)

    while not rospy.is_shutdown():
        if dataChanged:
            # - backward + forward
            vel_x  = cmdVel.Velocity
            # - ccw + cw
            vel_th = cmdVel.Turn 

            left_vel  = vel_x + vel_th * width_robot / 2.0
            right_vel = vel_x - vel_th * width_robot / 2.0
            
            total_vel = abs(left_vel) + abs(right_vel)
            
            #Prevent division by 0
            if total_vel != 0:
                #normalize velocities from a range -1 to 1
                left_norm  = left_vel  / total_vel
                right_norm = right_vel / total_vel

                #left_vel  = max(min(maxVel, left_vel), -maxVel)
                #right_vel = max(min(maxVel, right_vel),-maxVel)
            else:
                left_norm  = 0.0
                right_norm = 0.0

            #Publish velocities
            leftVelPub.publish(left_norm)
            rightVelPub.publish(right_norm)

            #print("%.6f %.6f" % (left_vel, right_vel))
            rospy.loginfo(left_vel)
            rospy.loginfo(right_vel)
            rospy.loginfo(left_norm)
            rospy.loginfo(right_norm)
            
            dataChanged = False

        r.sleep()


if __name__ == '__main__':
    try:
        BaseController()
    except rospy.ROSInterruptException: 
        pass
