#!/usr/bin/env python

import rospy
import roslib
import os
from gps_common.msg import GPSFix
from geometry_msgs.msg import Twist

directory = os.path.expanduser("~/catkin_ws/src/autonomous_mode/GPS_files/")
filename = "log.txt"

gpsData   = GpsFix()
gpsTarget = GpsFix()

def callback(data):
    #Get current GPS position from rover
    global gpsData
    gpsData.latitude  = data.latitude
    gpsData.longitude = data.longitude

def GpsPoints():
    
    global gpsdata
    global gpsTarget
    
    pub = rospy.Publisher('gpstarget', GPSFix, queue_size=10)
    rospy.init_node('GPSPoints', anonymous=True)
    rospy.Subscriber('GPS',GPSFix,callback)

    #open file with GPS coordinate points
    FILE = open(directory+filename,"r")
    points = []
    line = FILE.readline()
    while len(line) > 0:
        points.append(line.split())
        line = FILE.readline()
    FILE.close()
    print(points)
    
    curr_point = 0
    
    r = rospy.Rate(5) #5 Hz
    
    while not rospy.is_shutdown():
        
        #get lat, lon from current point
        gpsTarget.latitude  = points[curr_point][0]
        gpsTarget.longitude = points[curr_point][1]
        
        #publish gpsTarget
        pub.publish(gpsTarget)
        
        
        
        
        r.sleep()
        
    



if __name__ == '__main__':
    try:
        GpsPoints()

    except rospy.ROSInterruptException:
        pass


