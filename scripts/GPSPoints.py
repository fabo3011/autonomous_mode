#!/usr/bin/env python

import rospy
import roslib
import os
from gps_common.msg import GPSFix
from geometry_msgs.msg import Twist
from lib.GPSVector import *

#temp constants
minDistanceToFinalTarget = 5.00

#path where the list of GPS points [latitude, longitude] is located
directory = os.path.expanduser("~/catkin_ws/src/autonomous_mode/GPS_files/")
filename = "log.txt"

gpsData   = GPSFix()
gpsTarget = GPSFix()

#Points for function logic
gpsData_point   = GPSPoint()
gpsTarget_point = GPSPoint()

def callback(data):
    #Get current GPS position from rover
    global gpsData
    global gpsData_point
    gpsData.latitude  = data.latitude
    gpsData.longitude = data.longitude
    #Get GPSPoint for GPSData
    gpsData_point = GPSPoint(gpsData.latitude, gpsData.longitude)
    
#Logic Functions

#Load Points From File
#This funciton loads the list of points (pairs of GPS coordinates [latitude, longitude]) contained in pathToFile published by the GenerateList.py node
def loadPointsFromFile(pathToFile):
    points = []
    #open file with GPS coordinate points
    FILE = open(pathToFile,"r")
    #read first line
    line = FILE.readline()
    while len(line) > 0:    #while there exists a next line to read
        points.append(line.split())
        line = FILE.readline()
    #close file
    FILE.close()
    #return list of points
    return points

#function to test if the rover has arrived to the target (within minDistanceToFinalTarget range)
def arrivedToFinalTarget(data, target):

    #Calculate vector taking data and target as points
    dest_vec = GPSVector(data,target)
    
    print(dest_vec)
    
    #Test if distance is less than minDistanceToFinalTarget parameter
    if dest_vec.magnitude <= minDistanceToFinalTarget:
        print('Houston, the rover has arrived ... ')
        print('SUCCESS')
        return True
    else:
        return False

#function to test if its closer to the next target in the list    
def closerToNextTarget(data, target, next_target):
    
    #Calculate vector taking data and target as points
    dest_vec = GPSVector(data,target)
    
    print(dest_vec)
    
    #Calcualte vector between next target and data
    next_to_data = GPSVector(next_target, data)
    
    #Calcualte vector between next target and current target
    next_to_target = GPSVector(next_target, target)
    
    #Compare distances and checks if data is closer to next target
    if next_to_data.magnitude <= next_to_target.magnitude:
        print('Target Reached!')
        print('SUCCESS')
        return True
    else:
        return False
    
    
def bye():
    print('Autonomous Mission Ended, see you soon EagleX...')

def GpsPoints():
    
    global gpsData
    global gpsTarget
    
    global gpsData_point
    global gpsTarget_point
    
    pub = rospy.Publisher('gpstarget', GPSFix, queue_size=10)
    rospy.init_node('GPSPoints', anonymous=True)
    rospy.Subscriber('GPS',GPSFix,callback)
    rospy.on_shutdown(bye)

    #obtain list of points from file
    points = loadPointsFromFile(directory+filename)
    
    #index that points to the current point to publish in /gpstarget
    curr_point_idx = 0
    
    r = rospy.Rate(4) #5 Hz (rate of /gpstarget)
    
    while not rospy.is_shutdown():
        
        #get [lat, lon] from current point
        if curr_point_idx != len(points):
            gpsTarget.latitude  = float(points[curr_point_idx][0])
            gpsTarget.longitude = float(points[curr_point_idx][1])
        #Get GPSPoint for GPSTarget
        gpsTarget_point = GPSPoint(gpsTarget.latitude, gpsTarget.longitude)
        #publish gpsTarget
        pub.publish(gpsTarget)
        
        print(curr_point_idx)
        
        if curr_point_idx == len(points)-1:     #target is the last one
            
            #function to evaluate if its close enough to arrive to the final target
            if arrivedToFinalTarget(gpsData_point, gpsTarget_point):
                gpsTarget.latitude  = 0.0
                gpsTarget.longitude = 0.0
                pub.publish(gpsTarget)
                curr_point_idx += 1
                
        elif curr_point_idx == len(points):     #final target has been reached
            continue    
        else:
            #check if its closer to the next target in the list
            next_target_point = GPSPoint(float(points[curr_point_idx+1][0]), float(points[curr_point_idx+1][1]))
            if closerToNextTarget(gpsData_point, gpsTarget_point, next_target_point):
                curr_point_idx += 1
            
        r.sleep()
        
    



if __name__ == '__main__':
    try:
        GpsPoints()

    except rospy.ROSInterruptException:
        pass


