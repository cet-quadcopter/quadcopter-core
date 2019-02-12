#!/usr/bin/env python
# for altitude lock in airsim simulation
import rospy
import time
from drone_std_msgs.msg import Propeller, GPS, Barometer
from drone_commands.msg import Movement

#pressure=997
altitude = 123340
count = 0
counter2 = 0
roll_cmd=0
def set_altitude(motor_inp,var):
    
    #rospy.init_node('altitude_pub',anonymous = True)
    rate = rospy.Rate(10) # 10hz
    a = Propeller()
    a.prop1 = motor_inp+var
    a.prop2 = motor_inp
    a.prop3 = motor_inp
    a.prop4 = motor_inp+var
    rospy.loginfo(a)
    pub.publish(a)
    rate.sleep()

def callback(baro):
    global altitude
    altitude = float(baro.alt)
    

def gps_subscriber():
    rospy.init_node('altitude_pub',anonymous = False)
    #rospy.init_node('gps_sub', anonymous=True)
    rospy.Subscriber('/sensors/gps', GPS, callback)



def call_back(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.command_num)
    global roll_cmd
    roll_cmd = data.command_num
def listener(): 
    
    rospy.Subscriber('command', Movement, call_back) 




if __name__ == '__main__':
    
    que=[0,0]
    gps_subscriber()
    listener()
    kp=2.5
    ki=0.0005
    kd=50
    set_point = 150000
    p = 1 
    pub = rospy.Publisher('/actuators/propeller', Propeller, queue_size = 10)
    while not rospy.is_shutdown():
            #gps_subscriber()
	#e=pressure-set_point
	e=(set_point-altitude)/5000
	que.append(e)
	if len(que)>20:
	   del que[0]
	sum=reduce(lambda s,v:s+v,que,0)
        actuation=kp*e+ki*sum+kd*(e-que[-2])
	diff=(e-que[-2])
	if actuation<0:
           actuation=.4
        #if pressure < set_point:
         #       a = 0.48
        #else:
               # a = 0.5
        set_altitude(actuation,0)
        count += 1
        #print "pressure %s" %pressure
        print "count: %s" %count 
	print "sum: %s" %sum
	print "error: %s" %e
 	print "Altitude:%s"%altitude
	print "difference:%s"%diff
        global roll_cmd
        if roll_cmd == 1:
           
           set_altitude(actuation,0.01*actuation)
	   time.sleep(.1)
           set_altitude(actuation,-.01*actuation)
    	   time.sleep(.1)
           set_altitude(actuation,0)
	   roll_cmd=0
           
       
	   
	   
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
        

    

    


