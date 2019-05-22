#!/usr/bin/env python

#node by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

#Project ALVIN (Autonomous Logic Virtual Intelligence n' Navigation)

import rospy
from nav2d_operator.msg import cmd
from std_msgs.msg import Float32


#constant limit sets
maxVel = 1.0
maxTurn = 0.6

#base controller constants
width_robot =  1.5

#global variables
cmdVel = cmd()

def cmdCb(data):
    global cmdVel
    cmdVel.Velocity = data.Velocity
    cmdVel.Turn     = data.Turn



def BaseController():

    global cmdVel

    pub = rospy.Publisher('cmd', cmd, queue_size=10)

    leftVelPub  = rospy.Publisher('/BaseController/left_vel',  Float32, queue_size=10)
    rightVelPub = rospy.Publisher('/BaseController/right_vel', Float32, queue_size=10)
    rospy.init_node('BaseController', anonymous=True)
    rospy.Subscriber('cmd',cmd,cmdCb)


    r = rospy.Rate(5) #5 Hz (rate of /gpstarget)

    while not rospy.is_shutdown():

        vel_x  = cmdVel.Velocity
        vel_th = cmdVel.Turn 

        left_vel  = vel_x + vel_th * width_robot / 2.0
        right_vel = vel_x - vel_th * width_robot / 2.0

        left_vel  = max(min(maxVel, left_vel), -maxVel)
        right_vel = max(min(maxVel, right_vel),-maxVel)

        leftVelPub.publish(left_vel)
        rightVelPub.publish(right_vel)

        print("%.6f %.6f" % (left_vel, right_vel))

        r.sleep()


if __name__ == '__main__':
    try:
        BaseController()
    except rospy.ROSInterruptException: 
        pass
