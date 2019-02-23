#!/usr/bin/env python

import rospy
import roslib
import os
from gps_common.msg import GPSFix
from geometry_msgs.msg import Twist

directory = os.path.expanduser("~/catkin_ws/src/autonomous_mode/GPS_files/")
filename = "log.txt"

def GpsPoints():
    
    pub = rospy.Publisher('gpstarget', GPSFix, queue_size=10)
    rospy.init_node('GPSPoints', anonymous=True)

    #open file
    FILE = open(directory+filename,"r")
    points = []
    line = FILE.readline()
    while len(line) > 0:
        points.append(line.split())
        line = FILE.readline()
    FILE.close()
    print(points)
    



if __name__ == '__main__':
    try:
        GpsPoints()

    except rospy.ROSInterruptException:
        pass


