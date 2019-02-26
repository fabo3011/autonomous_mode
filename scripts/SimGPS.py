#!/usr/bin/env python
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
directory = os.path.expanduser("~/catkin_ws/src/autonomous_mode/GPS_files/")
filename = "sim_init.txt"

def pose_callback(data):
    global pose
    pose.pose.pose.position.x  = data.pose.pose.position.x
    pose.pose.pose.position.y  = data.pose.pose.position.y 
    pose.pose.pose.orientation = data.pose.pose.orientation   

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

def SimGPS():

    global pose

    #data to be published
    gpsData = GPSFix()

    pub = rospy.Publisher('gpssim', GPSFix, queue_size=10)
    rospy.init_node('SimGPS', anonymous=True)
    rospy.Subscriber('base_pose_ground_truth',Odometry,pose_callback)

    #load initial point (lat = init_point[0][0], lon = init_point[0][1])
    init_point_file = loadPointsFromFile(directory+filename)

    #load initial point to an object GPSPoint()
    origin = GPSPoint(float(init_point_file[0][0]),float(init_point_file[0][1]))

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
