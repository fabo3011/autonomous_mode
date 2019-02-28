#!/usr/bin/env python

#node by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

#Project ALVIN (Autonomous Logic Virtual Intelligence n' Navigation)

import rospy
import roslib
import os
from gps_common.msg import GPSFix
from std_msgs.msg import String
from lib.GPSVector import *

#variable to cath gps data
gpsData = None
#flag to identify the first gps frame that goes into the callback function
firstGPSPoint = 0
#GPSPoint to store the target
target = None

def callback(data):
    global gpsData
    global firstGPSPoint
    if firstGPSPoint == 0:
        gpsData = GPSPoint(data.latitude, data.longitude)
        firstGPSPoint = 1
        print("First GPS frame detected %s\n" % (gpsData))


def getTarget():
    
    global target
    #Read target from user and store it on GPSPoint target
    target_str = raw_input('Target GPS Coordinates [lat, long] (decimal degrees): ')
    print(target_str)
    gpsTmp = target_str.split()
    print(gpsTmp)
    target = GPSPoint(float(gpsTmp[0]), float(gpsTmp[1]))

    print("Target is: %s" % (target))


def bye():
    print('Autonomous Mission Ended, see you soon EagleX...')

def GpsGenerateList():

    global gpsData
    global firstGPSPoint

    pub = rospy.Publisher('GPSListDone', String, queue_size=10)
    rospy.init_node('GPSGenerateList', anonymous=True)
    rospy.Subscriber('GPS',GPSFix,callback)
    rospy.on_shutdown(bye)

    while not rospy.is_shutdown():
        
        if firstGPSPoint:
            #Generate list of points
            dataTargetVec = GPSVector(gpsData, target)
            points = dataTargetVec.getListOfIntermediatePointsBasedOnDistance(10)
            #Store it on file
            GPSList = GPSListOfPoints(points)
            GPSList.writeListToFile('log.txt')
            pub.publish('Done')
            exit(0)


if __name__ == '__main__':
    try:
        getTarget()
        GpsGenerateList()

    except rospy.ROSInterruptException:
        pass

