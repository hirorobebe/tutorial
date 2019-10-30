#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64

def controller_all(angle1, angle2, angle3, angle4, angle5):
    pub1 = rospy.Publisher('tilt1_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('tilt2_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('tilt3_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('tilt4_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('tilt5_controller/command', Float64, queue_size=10)
    rospy.init_node('controller_all', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rospy.loginfo("angle1=%f",angle1)
        rospy.loginfo("angle2=%f",angle2)
        rospy.loginfo("angle3=%f",angle3)
        rospy.loginfo("angle4=%f",angle4)
        rospy.loginfo("angle5=%f",angle5)
        pub1.publish(angle1)
        pub2.publish(angle2)
        pub3.publish(angle3)
        pub4.publish(angle4)
        pub5.publish(angle5)
        rate.sleep()

def limit(angle, LIM_ANG_P, LIM_ANG_M):
    if angle < LIM_ANG_M:
        angle_lim = LIM_ANG_M
    elif angle > LIM_ANG_P:
        angle_lim = LIM_ANG_P
    else:
        angle_lim = angle    
     return angle_lim  

def arm_control_top(x, y, z):
    L_12 = 0.076
    L_23 = 0.0825
    L_34 = 0.094 + 0.10
#    L_4H = 0.10

    R_24 = math.sqrt(x**2 + y**2)
    Z_24 = z 
    goal_length = math.sqrt(x**2+y**2+z**2)

#    if(goal_length > (L_23+ L_34 + L_4H)):
    if(goal_length > (L_23+ L_34)):
         print( "out of range" )

    theta_2d = math.acos((L_23**2)-(L_34**2)+(R_24**2)+(Z_24**2))/(2*L_23*math.sqrt((R_24**2)+(Z_24**2))))
    theta_3d = math.acos(((L_34**2)-(L_23**2)+(R_24**2)+(Z_24**2))/(2*L_34*math.sqrt((R_24**2)+(Z_24**2))))
#   theta_4d = math.atan2(Z_24, R_24);

    angle1 = math.atan2(x,y)
    angle2 = 0.0
    angle3 = theta_2d + theta_3d
    angle4 = 0.0
    angle5 = 0.0 

    LIM_ANG_P1 = 1.0
    LIM_ANG_M1 = 0.0
    LIM_ANG_P2 = 1.0
    LIM_ANG_M2 = 0.0
    LIM_ANG_P3 = 1.0
    LIM_ANG_M3 = 0.0
    LIM_ANG_P4 = 1.0
    LIM_ANG_M4 = 0.0
    LIM_ANG_P5 = 1.0
    LIM_ANG_M5 = 0.0 

    angle_lim1 = limit(angle1, LIM_ANG_P1, LIM_ANG_M1)
    angle_lim2 = limit(angle2, LIM_ANG_P2, LIM_ANG_M2)
    angle_lim3 = limit(angle3, LIM_ANG_P3, LIM_ANG_M3)
    angle_lim4 = limit(angle4, LIM_ANG_P4, LIM_ANG_M4)
    angle_lim5 = limit(angle5, LIM_ANG_P5, LIM_ANG_M5)

    controller_all(angle_lim1, angle_lim2, angle_lim3, angle_lim4, angle_lim5)
#    controller_all(angle1, angle2, angle3, angle4, angle5)

if __name__ == '__main__':
    x = 2.0
    y = 1.0
    z = 0.0
    try:
        arm_control_top(x, y, z)
    except rospy.ROSInterruptException:
        pass

####################################################################################

def class RuloBot():
    if use_arm:
        arm = arm_control_top(x, y, z)

def arm_control_top(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z