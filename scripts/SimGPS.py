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
from nav_msgs.msg import Odometry

#Package libraries
from lib.GPSVector import *
from lib.Quaternion import pyQuaternion


pose = Odometry()

#path where the initial GPS coordinate [latitude, longitude] is located
#directory = os.path.expanduser("~/catkin_ws/src/autonomous_mode/GPS_files/")
filename = "sim_init.txt"

def pose_callback(data):
    global pose
    pose.pose.pose.position.x  = data.pose.pose.position.x
    pose.pose.pose.position.y  = data.pose.pose.position.y 
    pose.pose.pose.orientation = data.pose.pose.orientation   

def SimGPS():

    global pose

    #data to be published
    gpsData = GPSFix()

    pub = rospy.Publisher('gpssim', GPSFix, queue_size=10)
    rospy.init_node('SimGPS', anonymous=True)
    rospy.Subscriber('base_pose_ground_truth',Odometry,pose_callback)

    #load initial point (lat = init_point[0][0], lon = init_point[0][1])
    init_point_file = GPSListOfPoints()
    init_point_file.loadListFromFile(filename)

    #load initial point to an object GPSPoint()
    origin = GPSPoint(init_point_file[0])

    r = rospy.Rate(1) #1 Hz (rate of /gpssim)

    while not rospy.is_shutdown():
        #Transform pose to XY point
        pose_point = XYPoint(pose.pose.pose.position.x, pose.pose.pose.position.y)
        #Transform pose_point to GPSPoint
        gpsData_point = pose_point.XYToGPS(origin)
        #Transform GPSPoint to GPSFix data ready to publish in topic
        gpsData.latitude  = gpsData_point.latitude
        gpsData.longitude = gpsData_point.longitude

        #Get track from quaternion
        quaternion = pyQuaternion()
        quaternion.ROSQuaternionTransform(pose.pose.pose.orientation)

        #Transform quaternion to jaw rotation (normalized to 360.0)
        gpsData.track = XYPoint(0.0,0.0).angleToBearing(quaternion.getJaw())

        #rospy.loginfo(gpsData)
        pub.publish(gpsData)

        r.sleep()


if __name__ == '__main__':
    try:
        SimGPS()

    except rospy.ROSInterruptException:
        pass
