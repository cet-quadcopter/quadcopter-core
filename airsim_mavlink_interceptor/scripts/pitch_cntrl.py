#!/usr/bin/env python
# for altitude lock in airsim simulation
import rospy
from drone_std_msgs.msg import Propeller, Gyro, Barometer

pressure = 0
count = 0
def set_pitch(thrust_inp, pitch_inp1, pitch_inp2):

    rate = rospy.Rate(10) # 10hz
    a = Propeller()
    a.prop1 = a.prop3 = thrust_inp + pitch_inp1 # front facing
    a.prop4 =  a.prop2 = thrust_inp + pitch_inp2 # rear facing
    rospy.loginfo(a)
    pub.publish(a)
    rate.sleep()

def callback(baro):
    
    global pressure
    pressure = float(baro.pabs)
    

def gps_subscriber():
    
    rospy.init_node('altitude_pub',anonymous = False)
    rospy.Subscriber('/sensors/barometer', Barometer, callback)

if __name__ == '__main__':
    
    que =[0,0] # for integral and derivative control data storage
    gps_subscriber()
    kp =  .45
    ki = .0005
    kd = 12
    set_point = 997.0
    pub = rospy.Publisher('/actuators/propeller', Propeller, queue_size = 10)
    while not rospy.is_shutdown():
	e = pressure-set_point
	que.append(e)
	if len(que)>30:  # limits the queue length 
	   del que[0]    # if length increases first value is deleted
	sum = reduce(lambda s,v:s+v,que)
        thrust = (kp*e)+ (ki*sum)+ kd*(e-que[-2])
        set_altitude(thrust)
        count += 1
        print "pressure %s" %pressure
        print "count: %s" %count  
	print "sum: %s" %sum
	print "error: %s" %e