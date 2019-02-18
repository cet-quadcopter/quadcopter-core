#!/usr/bin/env python
# for altitude lock in airsim simulation
import rospy
import time
from drone_std_msgs.msg import Propeller, GPS, Barometer,Gyro
from drone_commands.msg import Movement

#pressure=997
altitude = 123340
count = 0
cmd=0

array_roll  = [0,0]
array_pitch = [0,0]
array_yaw   = [0,0]

yaw=0
pitch=0
roll=0

array_time = [0]


def set_altitude(motor_inp1,motor_inp2,motor_inp3,motor_inp4):
    
    #rospy.init_node('altitude_pub',anonymous = True)
    rate = rospy.Rate(10) # 10hz
    a = Propeller()
    a.prop1 = motor_inp1
    a.prop2 = motor_inp2
    a.prop3 = motor_inp3
    a.prop4 = motor_inp4
    rospy.loginfo(a)
    pub.publish(a)
    rate.sleep()

def callback(baro):
    global altitude
    altitude = float(baro.alt)

def cmd_callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.command_num)
    global cmd
    cmd = data.command_num   

def gyro_callback(angle):    

    global array_roll ,array_pitch,array_yaw,pitch,roll,yaw,array_time

    array_roll.append(angle.gy)
    array_pitch.append(angle.gx)
    array_yaw.append(angle.gz)

    sum_roll=reduce(lambda s,v:s+v,array_roll)
    sum_pitch=reduce(lambda s,v:s+v,array_pitch)
    sum_yaw=reduce(lambda s,v:s+v,array_yaw)
    
    if len(array_time)>2:
        del array_time[0]

    array_time.append(rospy.get_time())

    interval = array_time[1]-array_time[0]
    
    pitch = (((sum_pitch*interval*(1260/22))+180)%360)-180
    roll  = (((sum_roll*interval*(1260/22))+180)%360)-180
    yaw   = (((sum_yaw*interval*(1260/22))+180)%360)-180
    
    
def gps_subscriber():

    rospy.init_node('altitude_pub',anonymous = False)
    rospy.Subscriber('/sensors/gps', GPS, callback)

def listener():

    rospy.Subscriber('command', Movement, cmd_callback)
    
def gyro():

    rospy.Subscriber('/sensors/gyro', Gyro, gyro_callback)     
    
if __name__ == '__main__':
    
    que=[0,0]
    que_roll=[0,0]    
    gps_subscriber()
    gyro()
    listener()
    kp=2.5
    ki=0.0005
    kd=50
    kp_roll=.0001
    kd_roll=.001
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
        set_altitude(actuation,actuation,actuation,actuation)
        count += 1
        #print "pressure %s" %pressure
        print "count: %s" %count  
	print "sum: %s" %sum
	print "error: %s" %e
 	print "Altitude:%s"%altitude
	print "difference:%s"%diff
        print "yaw: %s"%yaw
        print "pitch: %s" %pitch
        print "roll: %s" %roll
        
        if cmd == 1:
          # set_altitude(actuation,actuation+.001*actuation,actuation,actuation+.001*actuation)
	  # time.sleep(1)
	  # set_altitude(actuation,actuation-.001*actuation,actuation,actuation-.001*actuation)
    	  # time.sleep(1)
	   
           set_roll = 50
           while not((set_roll-roll)<2):
           
               err_roll = set_roll-roll
               que_roll.append(err_roll)
	       if len(que)>20:
	          del que_roll[0]
	       sum_roll=reduce(lambda s,v:s+v,que_roll)
               control_roll = kp_roll*err_roll+kd_roll*(err_roll-que_roll[-2])
               set_altitude(actuation,actuation+control_roll,actuation,actuation+control_roll)
               print "roll: %s" %roll
               print "control_roll: %s" %control_roll

           cmd=0   
        elif cmd == 2:
           
             set_altitude(actuation,actuation,actuation+.01*actuation,actuation+.01*actuation)
	     time.sleep(1)
	     set_altitude(actuation,actuation,actuation-.01*actuation,actuation-.01*actuation)
    	     time.sleep(1)
	     cmd=0


	   
	   
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
        

    

    


