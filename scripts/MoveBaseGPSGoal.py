#!/usr/bin/env python

#node by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

#Command r t
# r -> distance in m from the final target at which ball_tracker algorithm is turned on
# t -> topic to which read data from (sim is from simulation, gps for real data)

#Project ALVIN (Autonomous Logic Virtual Intelligence n' Navigation)

import rospy
import roslib
import sys
from gps_common.msg import GPSFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import UInt8
from nav2d_operator.msg import cmd
from lib.GPSVector import *

# command flags
# r -> distance in m from the final target at which ball_tracker algorithm is turned on
#Defualt 20 m
distanceFromTargetToTurnOnBallTracker = 20
# t -> topic to which read data from (sim is from simulation, gps for real data)
#default sim
gpsTopic = "sim"



#limits for lineal velocity & turn
maxVel = 2.0  #2.0 m/s
minVel = 0.3  #1.8 m/s
maxTurn = 0.8 #0.6 rad/s

Kv = 0.2      #Constant for linear velocity
Kt = 2        #Constant for turn velocity

gpsError = 5  #Max error possible for GPS

#GPSFix to cache the GPS data
gpsData = GPSFix()

#GPSPoint to cache the GPS data
gpsDataPoint = GPSPoint()

#GPSPoint to cache the GPSTarget
target  = GPSPoint()

#GPSPoint to cache the GPSTarget
finalTarget  = GPSPoint()

#flag to indicate if final goal has been reached
goal = False

#ball controller status 
# 0 ball tracker takes control
# 1 I have the control again
# 2 ball found
ballControllerStatus = UInt8(1)

#gps controller status
# 0 ball traker disabled
# 1 ball tracker enabled
gpsControllerStatus = UInt8(0)

#program status
# 1 alive
# 0 dead
programStatus = 1

#cmd publisher
pub_cmd = None

#flag to detect first frame in gps's position
firstGPSCb = False

def GPSCallback(data):
    global gpsData
    global gpsDataPoint
    global firstGPSCb

    #Set flag to true
    firstGPSCb = True
    
    gpsData = data
    gpsDataPoint = GPSPoint(gpsData.latitude, gpsData.longitude) 
    print("on cb %s\n" % (gpsDataPoint))

def targetCallback(data):
    global target
    target = GPSPoint(data.latitude, data.longitude)

def finalTargetCallback(data):
    global finalTarget
    finalTarget = GPSPoint(data.latitude, data.longitude)

def msgCb(data):
    global minVel
    global goal
    if data.data == 'loading':
        minVel = 0.0
    elif data.data == 'done':
        goal = True
        
def ballControllerCallback(data):

    global gpsControllerStatus
    global ballControllerStatus
    global programStatus
    global pub_cmd

    #If ball tracker is disabled, ignore message
    if gpsControllerStatus == 0:
        return

    #Ball tracker wants to take control
    if data.data == 0:
        #Stop Rover
        gpsCmd = cmd()
        gpsCmd.Velocity = 0.0
        gpsCmd.Turn     = 0.0
        pub_cmd.publish(gpsCmd)
        #Kill program
        programStatus = 0
        print("Ball tracker took control")
    elif data.data == 1:
        programStatus = 1
        print("Control Returned")
    elif data.data == 2:
        print('SUCCESS: Mission accomplished.')
        programStatus = 0
        exit(0)
    
    ballControllerStatus = data

def getNodeInputs():
    global distanceFromTargetToTurnOnBallTracker
    global gpsTopic

    numberOfArguments = len(sys.argv)
    print(numberOfArguments)
    if   numberOfArguments == 1:
        return
    elif numberOfArguments == 2:
        distanceFromTargetToTurnOnBallTracker = float(sys.argv[1])
    elif numberOfArguments == 3:
        distanceFromTargetToTurnOnBallTracker = float(sys.argv[1])
        gpsTopic = sys.argv[2]
    else:
        print("Invalid number of arguments in input")


def MoveBaseGPSGoal():

    #GPS global variables
    global gpsData
    global target
    global finalTarget
    global gpsDataPoint
    global gpsError

    #Sys argv flags
    global distanceFromTargetToTurnOnBallTracker
    global gpsTopic

    #Global Publisher
    global pub_cmd

    #Goal flag
    global goal

    #Global status variables
    global gpsControllerStatus
    global ballControllerStatus
    global programStatus

    #cmd publisher
    pub_cmd = rospy.Publisher('cmd', cmd, queue_size=10)

    #ball tracker publisher
    pub_ball = rospy.Publisher('gps_controller', UInt8, queue_size=10)

    #Node init
    rospy.init_node('MoveBaseGPSGoal', anonymous=True)

    #Suscriber with next target in list to be acheived
    rospy.Subscriber('gpstarget', GPSFix, targetCallback)

    #Subscriber with GPS current position (based on GpsTopic select between gpssim and GPS)
    if gpsTopic == "sim":
        rospy.Subscriber('gpssim', GPSFix, GPSCallback)
    elif gpsTopic == "gps":
        rospy.Subscriber('GPS', GPSFix, GPSCallback)
    else:
        print("Invalid flag name for gps topic suscriber")
        exit(0)

    #Topic that indicates the status regarding to the final target
    rospy.Subscriber('onfinaltarget', String, msgCb)

    #Suscriber with final target (leg's gps coordinate) to be achieved
    rospy.Subscriber('finaltarget', GPSFix, finalTargetCallback)

    #Suscriber to communicate with ball tracker algorithm
    rospy.Subscriber('ball_controller', UInt8, ballControllerCallback)

    gpsCmd = cmd()

    r = rospy.Rate(5) #5 Hz (rate of /gpstarget)

    while not rospy.is_shutdown():

        if goal:
            print('Final GPS coordinate reached!!!')
            gpsCmd.Velocity = 0.0
            gpsCmd.Turn     = 0.0
            pub_cmd.publish(gpsCmd)
            programStatus = 0
            #exit(0)

        if programStatus == 1 and firstGPSCb:

            #Ball tracker enabler
            gpsToFinalTarget = GPSVector(gpsDataPoint,finalTarget)
            print("Magnitude -------> %f <= %f" % (gpsToFinalTarget.magnitude, distanceFromTargetToTurnOnBallTracker))
            print("status --> %d" % (gpsControllerStatus.data))
            if gpsToFinalTarget.magnitude <= distanceFromTargetToTurnOnBallTracker and gpsControllerStatus.data == 0:
                gpsControllerStatus = UInt8(1)
                pub_ball.publish(gpsControllerStatus)
            elif gpsToFinalTarget.magnitude >= distanceFromTargetToTurnOnBallTracker + gpsError and gpsControllerStatus.data == 1:
                gpsControllerStatus = UInt8(0)
                pub_ball.publish(gpsControllerStatus)

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
            pub_cmd.publish(gpsCmd)

        r.sleep()

if __name__ == '__main__':
    try:
        #Processes flags called when executing the command
        getNodeInputs()
        MoveBaseGPSGoal()

    except rospy.ROSInterruptException:
        pass
