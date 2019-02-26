#!/usr/bin/env python

import math
import rospy
import time

from drone_std_msgs.msg import Gyro, angles2

R = 0
G = Gyro()


def callback(Gyro):
    global R
    global G

    G = Gyro
    

def gyro_subscriber():
    rospy.init_node('angle_pub_gyro',anonymous = False)
    rospy.Subscriber('/sensors/gyro', Gyro, callback)


if __name__ == '__main__':
    #angle = angles2()
    gyro_subscriber()
    #pub = rospy.Publisher('/status/angles2', angles2, queue_size = 10)
    count = 0
    prevtime = time.time()
    while not rospy.is_shutdown():
        c = time.time()
        dt = prevtime- c
        pitch = G.gx*dt*180/math.pi
        roll = G.gy*dt*180/math.pi
        print "roll:%s"%roll
        print "pitch:%s"%pitch
        count += 1
        print "count:%s"%count
        prevtime = time.time()


        