#!/usr/bin/env python

#node by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

#Project ALVIN (Autonomous Logic Virtual Intelligence n' Navigation)

import rospy
import roslib
import os
from gps_common.msg import GPSFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from lib.GPSVector import *

#temp constants
minDistanceToFinalTarget = 2.00 # 2m
minDistanceToOtherTarget = 2.00 # 2m

#path where the list of GPS points [latitude, longitude] is located
#directory = os.path.expanduser("~/catkin_ws/src/autonomous_mode/GPS_files/")
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
    if next_to_data.magnitude <= next_to_target.magnitude or dest_vec.magnitude <= minDistanceToOtherTarget:
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
    
    pub        = rospy.Publisher('gpstarget', GPSFix, queue_size=10)
    pub_target = rospy.Publisher('onfinaltarget', String, queue_size=10)
    rospy.init_node('GPSPoints', anonymous=True)
    rospy.Subscriber('GPS',GPSFix,callback)
    rospy.on_shutdown(bye)

    #obtain list of points from file
    points = GPSListOfPoints()
    points.loadListFromFile(filename)
    
    #index that points to the current point to publish in /gpstarget
    curr_point_idx = 0
    
    r = rospy.Rate(4) #5 Hz (rate of /gpstarget)
    
    while not rospy.is_shutdown():
        
        #get [lat, lon] from current point
        if curr_point_idx != len(points):
            gpsTarget.latitude  = points[curr_point_idx].latitude
            gpsTarget.longitude = points[curr_point_idx].longitude
        #Get GPSPoint for GPSTarget
        gpsTarget_point = GPSPoint(gpsTarget.latitude, gpsTarget.longitude)
        #publish gpsTarget
        pub.publish(gpsTarget)
        
        print(curr_point_idx)
        
        if curr_point_idx == len(points)-1:     #target is the last one
            
            pub_target.publish('loading')
            #function to evaluate if its close enough to arrive to the final target
            if arrivedToFinalTarget(gpsData_point, gpsTarget_point):
                gpsTarget.latitude  = 0.0
                gpsTarget.longitude = 0.0
                pub.publish(gpsTarget)
                curr_point_idx += 1
                pub_target.publish('done')
                
        elif curr_point_idx == len(points):     #final target has been reached
            exit(0)
            pass    
        else:
            #check if its closer to the next target in the list
            next_target_point = GPSPoint(points[curr_point_idx+1])
            if closerToNextTarget(gpsData_point, gpsTarget_point, next_target_point):
                curr_point_idx += 1
            
        r.sleep()
        
    



if __name__ == '__main__':
    try:
        GpsPoints()

    except rospy.ROSInterruptException:
        pass


