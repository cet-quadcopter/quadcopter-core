#!/usr/bin/env python
# for altitude lock in airsim simulation
import rospy
from drone_std_msgs.msg import Propeller, GPS, Barometer


pressure = 997.0
count = 0
def set_altitude(motor_inp):
    
    #rospy.init_node('altitude_pub',anonymous = True)
    rate = rospy.Rate(10) # 10hz
    a = Propeller()
    a.prop1 = motor_inp
    a.prop2 = motor_inp
    a.prop3 = motor_inp
    a.prop4 = motor_inp
    rospy.loginfo(a)
    pub.publish(a)
    rate.sleep()

def callback(baro):
    global pressure
    pressure = float(baro.pabs)
    

def gps_subscriber():
    rospy.init_node('altitude_pub',anonymous = False)
    #rospy.init_node('gps_sub', anonymous=True)
    rospy.Subscriber('/sensors/barometer', Barometer, callback)

if __name__ == '__main__':
    
    que=[0,0]
    gps_subscriber()
    kp=.4
    ki=.0005
    kd=10
    set_point = 997.0
    p = 1 
    pub = rospy.Publisher('/actuators/propeller', Propeller, queue_size = 10)
    while not rospy.is_shutdown():
            #gps_subscriber()
	e=pressure-set_point
	que.append(e)
	if len(que)>30:
	   del que[0]
	sum=reduce(lambda s,v:s+v,que,0)
        actuation=kp*e+ki*sum+kd*(e-que[-2])
        #if pressure < set_point:
         #       a = 0.48
        #else:
               # a = 0.5
        set_altitude(actuation)
        count += 1
        print "pressure %s" %pressure
        print "count: %s" %count  
	print "sum: %s" %sum
	print "error: %s" %e
    
    #while True:
    #    gps_subscriber()
        
    #    if altitude > set_point:
    #        a = 0.3
    #    else:
    #        a= 0.5
    #    set_altitude(a)
    #    count += 1
    #    print "altitude: %s" %altitude
    #    print "count: %s" %count
        

    

    


