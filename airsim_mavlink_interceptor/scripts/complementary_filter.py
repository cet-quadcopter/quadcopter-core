#!/usr/bin/env python

import math
import rospy
import time
from drone_std_msgs.msg import Accelerometer, Gyro, angles

R = Gyro()
G = Accelerometer()
prevroll = 0
prevpitch = 0
alpha = 0.7


def callback(accelero):
    global G
    G = accelero

def callback2(gyro):
    global R
    R = gyro   

def accelero_subscriber():
    rospy.init_node('angle_pub',anonymous = False)
    rospy.Subscriber('/sensors/accelerometer', Accelerometer, callback)
    
def gyro_subscriber():
    rospy.Subscriber('/sensors/gyro', Gyro, callback2)

if __name__ == '__main__':
    angle = angles()
    accelero_subscriber()
    count = 0
    #pub = rospy.Publisher('/status/angles', angles, queue_size = 10)
    global prevpitch
    global prevroll
    prevtime = time.time()
    while not rospy.is_shutdown():
        a = math.atan2(G.ay,G.az)*(180/math.pi)
        b = math.atan2(-G.ax,(math.sqrt(G.ay**2 + G.az**2)))*(180/math.pi)
        c = time.time()
        dt = prevtime- c
        angle.roll = (((1-alpha)*(prevroll + (R.gx*dt*180/math.pi))) + alpha*a)
        angle.pitch = (((1-alpha)*(prevpitch + (R.gy*dt*180/math.pi))) + alpha*b) 
        prevroll = angle.roll
        prevpitch = angle.pitch
        #rospy.sleep(1)
        #pub.publish(angle)
        print "roll:%s"%angle.roll
        print "pitch:%s"%angle.pitch
        count += 1
        print "count:%s"%count
        prevtime = time.time()
        rospy.sleep(1)
        

