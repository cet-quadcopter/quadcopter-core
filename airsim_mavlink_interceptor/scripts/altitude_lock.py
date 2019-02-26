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
cmd=0
def set_altitude(motor_inp,var1=0,var2=0,var3=0):
   rate = rospy.Rate(10) # 10hz
   a = Propeller()
   a.prop1 = motor_inp+var1+var2+var3
   a.prop2 = motor_inp+var3
   a.prop3 = motor_inp+var2
   a.prop4 = motor_inp+var1
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
   global cmd
   cmd = data.command_num

def listener():
   rospy.Subscriber('command', Movement, call_back) 

if __name__ == '__main__':
   que=[0,0]
   gps_subscriber()
   listener()
   kp=2.5
   ki=0.00055
   kd=50
   set_point = 150000
   p = 1 
   pub = rospy.Publisher('/actuators/propeller', Propeller, queue_size = 10)
   while not rospy.is_shutdown():
      e=(set_point-altitude)/5000
      que.append(e)
      if len(que)>20:
         del que[0]
      
      sum=reduce(lambda s,v:s+v,que,0)
      actuation=kp*e+ki*sum+kd*(e-que[-2])
      diff = (e-que[-2])
      if actuation<0:
         actuation=.45

      set_altitude(actuation,0)
      count += 1
      #print "pressure %s" %pressure
      print "count: %s" %count
      print "sum: %s" %sum
      print "error: %s" %e
      print "Altitude:%s"%altitude
      print "difference:%s"%diff
      global cmd
        
      if cmd == 1:
         set_altitude(actuation,0.01*actuation)
         time.sleep(.01)
         set_altitude(actuation,-0.01*actuation)
         time.sleep(.01)
         set_altitude(actuation,0,0)
         cmd=0
      elif cmd == 3:
         set_altitude(actuation,-0.01*actuation)
         time.sleep(.01)
         set_altitude(actuation,0.01*actuation)
         time.sleep(.01)
         set_altitude(actuation,0,0)
         cmd=0
        
      elif cmd == 8: #pitch control backward
         set_altitude(actuation,0,0.01*actuation)
         time.sleep(.01)
         set_altitude(actuation,0,-0.01*actuation)
         time.sleep(.01)
         set_altitude(actuation,0,0)
         cmd=0

      elif cmd == 2:
         set_altitude(actuation,0,-0.01*actuation)
         time.sleep(.01)
         set_altitude(actuation,0,0.01*actuation)
         time.sleep(.01)
         set_altitude(actuation,0,0)
         cmd=0

      elif cmd == 5:
         set_altitude(actuation,0,0,0.05*actuation)
         time.sleep(.5)
         #set_altitude(actuation,0,0,-0.01*actuation)
	      #time.sleep(.1)
         set_altitude(actuation,0,0,0)
         cmd=0

      elif cmd == 4:
         set_altitude(actuation,0,0,-0.05*actuation)
         time.sleep(.5)
         #set_altitude(actuation,0,0,-0.01*actuation)
	      #time.sleep(.1)
         set_altitude(actuation,0,0,0)
         cmd=0

