#!/usr/bin/env python

import rospy
from drone_std_msgs.msg import Propeller

def actuator():

    pub = rospy.Publisher('/actuators/propeller', Propeller, queue_size = 10)
    rospy.init_node('motor', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        motor_input = Propeller()
	motor_input.prop1=a1
	motor_input.prop2=a2
	motor_input.prop3=a3
	motor_input.prop4=a4
        rospy.loginfo(motor_input)
        pub.publish(motor_input)
        rate.sleep()

if __name__ == '__main__':
    try:
	global a1,a2,a3,a4
	a1 = input(" propeller 1 signal ")
	a2 = input(" propeller 2 signal ")
	a3 = input(" propeller 3 signal ")
	a4 = input(" propeller 4 signal ")
        actuator()
    except rospy.ROSInterruptException:
        pass
