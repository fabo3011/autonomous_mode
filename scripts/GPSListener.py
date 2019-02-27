#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gps_common.msg import GPSFix
from lib.GPSVector import *

gpsData = GPSFix()
gpsList = GPSListOfPoints()
filename = 'log.txt'

#callback to cache GPS data (not fancy)
def callback(data):
    global gpsData
    global gpsList
    gpsData = data
    gpsDataPoint = GPSPoint(gpsData.latitude, gpsData.longitude)
    gpsList.addPointToList(gpsDataPoint)
    print(gpsDataPoint)
    #print(gpsList)

#store data one it has finished recording points
def store_data():
    global gpsList
    global filename
    #print(gpsList)
    gpsList.writeListToFile(filename)
    print('\ndata stored successfully')
    time = rospy.get_rostime()
    time_secs = time.to_sec()
    cachefilename = str(int(time_secs))+".txt"
    gpsList.writeListToFile(cachefilename)
    print('back up made successfully @ %s' % cachefilename)

def GPSListener():

    rospy.init_node('GPSListener',anonymous=True)
    rospy.Subscriber('GPS',GPSFix, callback)
    rospy.on_shutdown(store_data)

    r = rospy.Rate(1) #ROS Rate

    while not rospy.is_shutdown():
        pass
        #r.sleep()
    
if __name__=='__main__':
    GPSListener()
     