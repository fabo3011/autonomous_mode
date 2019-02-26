#!/usr/bin/env python

#lib by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

from math import *

import rospy
from geometry_msgs.msg import Quaternion

#---------------------Quaternion---------------------------------------------------------------------------------------------------------------------------

#JPL Quaternion implementation
class pyQuaternion:

    #Initializes Quaternion
    def __init__(self, q = None):

        #Default values
        if q == None:
            self.q = [0.0, 0.0, 0.0, 0.0]
            return

        #Handle TypeError exceptions when args are not instantiated correctly
        if not isinstance(q, list) or len(q) != 4:
            raise TypeError('Quaternion must be passed a list with 4 numbers')
        eps = 1e-6
        chck_sum = 0.0
        for i in range(0,4):
            if not isinstance(float(q[i]),float):
                raise TypeError('Elements in Quaternion must be numbers')    
            chck_sum += q[i]*q[i]
        #Check if the quaternions magnitude is equal to 1 with a tolerance of eps
        if chck_sum < 1.0-eps or chck_sum > 1.0+eps:
            raise TypeError('Quaternions magnitude must be equal to 1.0') 

        self.q = q

    #Transforms ROS Quaternion class to the object Quaternion
    def ROSQuaternionTransform(self, ROS_quaternion):

        #Handle TypeError exceptions when args are not instantiated correctly
        if not isinstance(ROS_quaternion, Quaternion):
            raise TypeError('ROSQuaternionTransform must be passed a geometry_msgs/Quaternion type object')

        self.q = [ float(ROS_quaternion.w), float(ROS_quaternion.x), float(ROS_quaternion.y), float(ROS_quaternion.z) ]

    #Get jaw from Quaternion (which is the most used rotation in the rover) in normalized degrees
    def getJaw(self):
        jaw = atan2( 2*(self.q[0]*self.q[3] + self.q[1]*self.q[2]), 1 - 2*(self.q[2]*self.q[2] + self.q[3]*self.q[3]) )
        return (degrees(jaw) + 360.0) % 360.0

    