#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64

def controller4():
    pub = rospy.Publisher('joint4_controller/command', Float64, queue_size=10)
    rospy.init_node('controller4', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        angle4 = 0.0
        rospy.loginfo(angle4)
        pub.publish(angle4)
        rate.sleep()

def controller5():
    pub = rospy.Publisher('joint5_controller/command', Float64, queue_size=10)
    rospy.init_node('controller5', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        angle5 = 0.0
        rospy.loginfo(angle5)
        pub.publish(angle5)
        rate.sleep()

def controller_all():
    controller5()
    controller4()

if __name__ == '__main__':
    try:
        controller_all()
    except rospy.ROSInterruptException:
        pass
