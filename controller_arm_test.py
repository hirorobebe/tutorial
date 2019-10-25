#!/usr/bin/env python
# license removed for brevity
import rospy
import math
#from std_msgs.msg import String
from std_msgs.msg import Float64

def controller1(theta1):
   pub = rospy.Publisher('chatter', Float64, queue_size=10)
   rospy.init_node('controller1', anonymous=True)
   rate = rospy.Rate(1) # 1hz
   while not rospy.is_shutdown():
#       theta1 = 0.5
       rospy.loginfo(theta1)
       pub.publish(theta1)
       rate.sleep()
       
def controller2(theta2):
   pub = rospy.Publisher('chatter', Float64, queue_size=10)
   rospy.init_node('controller2', anonymous=True)
   rate = rospy.Rate(1) # 1hz
   while not rospy.is_shutdown():
#       theta2 = 0.5
       rospy.loginfo(theta2)
       pub.publish(theta2)
       rate.sleep()

def controller3(theta3):
   pub = rospy.Publisher('chatter', Float64, queue_size=10)
   rospy.init_node('controller3', anonymous=True)
   rate = rospy.Rate(1) # 1hz
   while not rospy.is_shutdown():
#       theta3 = 0.5
       rospy.loginfo(theta3)
       pub.publish(theta3)
       rate.sleep()

def controller4(theta4):
   pub = rospy.Publisher('chatter', Float64, queue_size=10)
   rospy.init_node('controller4', anonymous=True)
   rate = rospy.Rate(1) # 1hz
   while not rospy.is_shutdown():
#       theta4 = 0.5
       rospy.loginfo(theta4)
       pub.publish(theta4)
       rate.sleep()

def controller5(theta5):
   pub = rospy.Publisher('chatter', Float64, queue_size=10)
   rospy.init_node('controller4', anonymous=True)
   rate = rospy.Rate(1) # 1hz
   while not rospy.is_shutdown():
#       theta5 = 0.5
       rospy.loginfo(theta5)
       pub.publish(theta5)
       rate.sleep()

def controller_all():
   L_12 = 0.076
   L_23 = 0.0825
   L_34 = 0.094 + 0.10
   x = raw_input('x: ')
   y = raw_input('y: ')
   z = raw_input('z: ')
   R_24 = math.sqrt(x**2 + y**2)
   Z_24 = z + L_12
   theta_2d = math.acos((L_23**2)-(L_34**2)+(R_24**2)+(Z_24**2))/(2*L_23*math.sqrt((R_24**2)+(Z_24**2))))
   theta_3d = math.acos(((L_34**2)-(L_23**2)+(R_24**2)+(Z_24**2))/(2*L_34*math.sqrt((R_24**2)+(Z_24**2))))
#   theta_4d = math.atan2(Z_24, R_24);
   theta1 = math.atan2(x,y)
   theta2 = 0
   theta3 = theta_2d + theta_3d
   theta4 = 0
   theta5 = 0
   controller1(theta1)
   controller2(theta2)
   controller3(theta3)
   controller4(theta4)
   controller5(theta5)
         
if __name__ == '__main__':
   try:
       controller_all()
   except rospy.ROSInterruptException:
       pass