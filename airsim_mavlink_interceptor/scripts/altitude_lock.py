#!/usr/bin/env python
# for altitude lock in airsim simulation
import rospy
import time
import numpy as np
from drone_std_msgs.msg import Propeller, GPS, Barometer, Gyro,Attitude
from drone_commands.msg import Movement

#pressure=997
altitude = 123340
count = 0
cmd = 0
control_roll=0
control_pitch=0
err_roll=0
array_roll = [0, 0]
array_pitch = [0, 0]
array_yaw = [0, 0]

yaw = 0
pitch = 0
roll = 0

array_time = [0]


def set_altitude(motor_inp1, motor_inp2, motor_inp3, motor_inp4):

    #rospy.init_node('altitude_pub',anonymous = True)
    rate = rospy.Rate(10)  # 10hz
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

    global array_roll, array_pitch, array_yaw, pitch, roll, yaw, array_time

    #array_roll.append(angle.gx)
    #array_pitch.append(angle.gy)
    #array_yaw.append(angle.gz)

    #sum_roll = reduce(lambda s, v: s+v, array_roll)
    #sum_pitch = reduce(lambda s, v: s+v, array_pitch)
    #sum_yaw = reduce(lambda s, v: s+v, array_yaw)

    #if len(array_time) > 2:
     #   del array_time[0]

    #array_time.append(rospy.get_time())

    #interval = array_time[1]-array_time[0]

    #pitch = (((sum_pitch*interval*(1260/22))+180) % 360)-180
    #roll = (((sum_roll*interval*(1260/22))+180) % 360)-180
    #yaw = (((sum_yaw*interval*(1260/22))+180) % 360)-180

    yaw   = angle.yaw
    pitch = angle.pitch
    roll  = angle.roll

def gps_subscriber():

    rospy.init_node('altitude_pub', anonymous=False)
    rospy.Subscriber('/sensors/gps', GPS, callback)


def listener():

    rospy.Subscriber('command', Movement, cmd_callback)


def gyro():

    #rospy.Subscriber('/sensors/gyro', Gyro, gyro_callback)
    rospy.Subscriber('/sensors/attitude',Attitude,gyro_callback)
    


if __name__ == '__main__':

    que = [0, 0]
    que_roll = [0, 0]
    que_pitch = [0, 0]
    que_yaw = [0, 0]
    gps_subscriber()
    gyro()
    listener()
    #global kp,kd,ki,kp_roll,kd_roll,ki_roll
    kp = 2.5
    ki = 0.02
    kd = 30

    kp_roll = .00001
    kd_roll = .001
    ki_roll = .00001

    kp_pitch = .00001
    kd_pitch = .001
    ki_pitch = .00001

    kp_yaw = .00001
    kd_yaw = .001
    ki_yaw = .00001

    set_point = 150000
    global p
    p = 0
    exit_roll = 0
    exit_pitch = 0
    exit_yaw   = 0
    pub = rospy.Publisher('/actuators/propeller', Propeller, queue_size=10)


    def altitude_lock(set_alt):                             #for altitude

        global actuation,e,diff
        e = (set_alt-altitude)/5000
        que.append(e)
        if len(que) > 20:
            del que[0]
        
        sum = reduce(lambda s, v: s+v, que, 0)
        actuation = kp*e+ki*sum+kd*(e-que[-2])
        diff = (e-que[-2])
        if actuation < 0:
            actuation = .4

        set_altitude(actuation, actuation, actuation, actuation)

    def roll_oper(set_roll_angle):                            # for roll control

        global control_roll,exit_roll,err_roll
        global cmd

        err_roll = set_roll_angle-roll
        que_roll.append(err_roll)
        if len(que_roll) > 5:
            del que_roll[0]
        sum_roll = reduce(lambda s, v: s+v, que_roll)
        control_roll = kp_roll*err_roll+kd_roll *(err_roll-que_roll[-2])+ki_roll*sum_roll
                        
        if control_roll > .001*actuation:
            control_roll = .001*actuation
        elif control_roll < (-.001*actuation):
            control_roll = -.001*actuation
        set_altitude(actuation-control_roll, actuation+control_roll,actuation+control_roll, actuation-control_roll)
                                    
        if set_roll_angle != 0 :
            if (np.abs(err_roll) < .2):
                print "sucess--------------------roll------------------------------------------"
                exit_roll = 1
                cmd = 0


    def pitch_oper(set_pitch_angle):                          #for pitch control

        global control_pitch,exit_pitch,err_pitch
        global cmd

        err_pitch = set_pitch_angle-pitch
        que_pitch.append(err_pitch)
        if len(que_pitch) > 5:
            del que_pitch[0]
        sum_pitch = reduce(lambda s, v: s+v, que_pitch)
        control_pitch = kp_pitch*err_pitch+kd_pitch *(err_pitch-que_pitch[-2])+ki_pitch*sum_pitch
                        
        if control_pitch > .001*actuation:
            control_pitch = .001*actuation
        elif control_pitch < (-.001*actuation):
            control_pitch = -.001*actuation
        set_altitude(actuation+control_pitch, actuation-control_pitch,actuation+control_pitch, actuation-control_pitch)
                                    
        if set_pitch_angle != 0 :
            if (np.abs(err_pitch) < .2):
                print "sucess-----------------pitch---------------------------------------------"
                exit_pitch = 1
                cmd = 0

    def yaw_oper(set_yaw_angle):                          #for yaw control

        global control_yaw,exit_yaw,err_yaw
        global cmd

        err_yaw = set_yaw_angle-yaw
        que_yaw.append(err_yaw)
        if len(que_yaw) > 5:
            del que_yaw[0]
        sum_yaw = reduce(lambda s, v: s+v, que_yaw)
        control_yaw = kp_yaw*err_yaw+kd_yaw *(err_yaw-que_yaw[-2])+ki_yaw*sum_yaw
                        
        if control_yaw > .001*actuation:
            control_yaw = .001*actuation
        elif control_yaw < (-.001*actuation):
            control_yaw = -.001*actuation
        set_altitude(actuation+control_yaw, actuation+control_yaw,actuation-control_yaw, actuation-control_yaw)
                                    
        if set_yaw_angle != 0 :
            if (np.abs(err_yaw) < .2):
                print "sucess-----------------yaw---------------------------------------------"
                exit_yaw = 1
                cmd = 0

    while not rospy.is_shutdown():
        
        altitude_lock(set_point) 
        count += 1
        
        if cmd == 1:

            exit_roll = 0
            roll_oper(5)
        
        if (exit_roll ==1) | (cmd == 0):

            roll_oper(0)

        if cmd == 2:

            exit_pitch = 0
            pitch_oper(5)
        
        if (exit_pitch ==1) | (cmd == 0):

            pitch_oper(0)

        if cmd == 3:

            exit_yaw = 0
            yaw_oper(5)
        
        if (exit_yaw ==1) | (cmd == 0):

            yaw_oper(0)
            
          
        print "count: %s" % count
        print "sum: %s" % sum
        print "error: %s" % e
        print "Altitude:%s" % altitude
        print "difference:%s" % diff
        print "yaw: %s" % yaw
        print "pitch: %s" % pitch
        print "roll: %s" % roll
        print "err_yaw: ---------------------------------%s" % err_yaw
        print "exit_yaw: -----------------------------------------%s" % exit_yaw
        print "cmd: -----------------------------------------%s" % cmd
        print "yaw_ctrl: -----------------------------------------%s" % control_yaw
        

        
       