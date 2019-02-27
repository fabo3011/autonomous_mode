#!/usr/bin/env python

#node by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

#Project ALVIN (Autonomous Logic Virtual Intelligence n' Navigation)

import rospy
import roslib
from gps_common.msg import GPSFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2d_operator.msg import cmd
from lib.GPSVector import *

#limits for lineal velocity & turn
maxVel = 2.0  #2.0 m/s
minVel = 1.8  #1.8 m/s
maxTurn = 0.6 #0.6 rad/s

Kv = 0.2      #Constant for linear velocity
Kt = 2        #Constant for turn velocity

#GPSFix to cache the GPS data
gpsData = GPSFix()

#GPSPoint to cache the GPS data
gpsDataPoint = GPSPoint()

#GPSPoint to cache the GPSTarget
target  = GPSPoint()

#flag to indicate if final goal has been reached
goal = False


def GPSCallback(data):
    global gpsData
    global gpsDataPoint
    gpsData = data
    gpsDataPoint = GPSPoint(gpsData.latitude, gpsData.longitude) 
    print("on cb %s\n" % (gpsDataPoint))

def targetCallback(data):
    global target
    target = GPSPoint(data.latitude, data.longitude)

def msgCb(data):
    global minVel
    global goal
    if data.data == 'loading':
        minVel = 0.0
    elif data.data == 'done':
        goal = True
        


def MoveBaseGPSGoal():

    global gpsData
    global target
    global gpsDataPoint

    pub = rospy.Publisher('cmd', cmd, queue_size=10)
    rospy.init_node('MoveBaseGPSGoal', anonymous=True)
    rospy.Subscriber('gpstarget',GPSFix,targetCallback)
    rospy.Subscriber('gpssim',GPSFix,GPSCallback)
    rospy.Subscriber('onfinaltarget',String,msgCb)

    gpsCmd = cmd()

    r = rospy.Rate(5) #5 Hz (rate of /gpstarget)

    while not rospy.is_shutdown():

        if goal:
            print('SUCCESS: Mission accomplished.')
            gpsCmd.Velocity = 0.0
            gpsCmd.Turn     = 0.0
            pub.publish(gpsCmd)
            exit(0)

        #Calculate lineal velocity and turn
        gpsToTarget = GPSVector(gpsDataPoint,target)
        #lineal velocity
        gpsCmd.Velocity = max(min(Kv*gpsToTarget.magnitude,maxVel), minVel)
        #angular velocity
        ang_diff = gpsToTarget.orientation-gpsData.track
        print(gpsDataPoint)
        print(gpsToTarget)
        #print(gpsToTarget.orientation)
        #print("ang_diff: %.6f" % (ang_diff))
        
        if ang_diff < 0.0:
            ccw = abs(ang_diff)
            cw  = ang_diff + 360.0
        else:
            cw  = ang_diff
            ccw = 360.0-ang_diff

        #Choose between clockwise and couter-clockwise turn
        if ccw < cw: #counter clock wise turn
            turn = -min(maxTurn,radians(ccw)) 
        else:   #clock wise turn
            turn = min(maxTurn,radians(cw)) 

        #assign turn to /cmd publish message    
        gpsCmd.Turn = turn
        #rospy.loginfo(gpsCmd)
        pub.publish(gpsCmd)

        r.sleep()

if __name__ == '__main__':
    try:
        MoveBaseGPSGoal()

    except rospy.ROSInterruptException:
        pass
