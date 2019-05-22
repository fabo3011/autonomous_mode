#!/usr/bin/env python

#node by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

#Project ALVIN (Autonomous Logic Virtual Intelligence n' Navigation)

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs.msg import Joy

#left vel
left_vel = Float32()
#right vel
right_vel = Float32()

maxVel = Float32() #2 m
maxVel.data = 1.0

def leftCb(data):
    global left_vel
    left_vel.data = data.data

def rightCb(data):
    global right_vel
    right_vel.data = data.data

def FakeJoy():

    global cmdVel

    pub  = rospy.Publisher('joy',  Joy, queue_size=10)
    rospy.init_node('FakeJoy', anonymous=True)
    rospy.Subscriber('/BaseController/left_vel',   Float32, leftCb)
    rospy.Subscriber('/BaseController/right_vel',  Float32, rightCb)


    r = rospy.Rate(5) #5 Hz (rate of /gpstarget)
    header = Header()
    axes = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    buttons = [0,0,0,0,0,0,0,0]
    joy = Joy(header,axes,buttons)
    print(joy)

    while not rospy.is_shutdown():
        #Set fake joy varibles
        joy.axes[1] = left_vel.data/maxVel.data
        joy.axes[4] = right_vel.data/maxVel.data
        #joy.frame_id = 5
        #print(joy)
        pub.publish(joy)

        r.sleep()


if __name__ == '__main__':
    try:
        FakeJoy()
    except rospy.ROSInterruptException: 
        pass
