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
pitch_desired = 0
roll_desired = 0
enter_lat = 0
err_latitude = 0
err_longitude = 0
latitude = 0
longitude = 0
pre_err_latitude = 0


speed_motor_1 = 0
speed_motor_2 = 0
speed_motor_3 = 0
speed_motor_4 = 0





def set_propeller(motor_inp1, motor_inp2, motor_inp3, motor_inp4):

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
    global altitude,latitude,longitude
    altitude = float(baro.alt)
    latitude = baro.lat
    longitude = baro.lon

def cmd_callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.command_num)
    global cmd
    cmd = data.command_num


def gyro_callback(angle):

    global  pitch, roll, yaw

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
    que_lat = [0,0]
    que_lon = [0,0]
    gps_subscriber()
    gyro()
    listener()
    #global enter_lat,err_latitude,pre_err_latitude

    kp = 2.5
    ki = 0.002
    kd = 10

    kp_roll = .00001
    kd_roll = .001
    ki_roll = .00001

    kp_pitch = .00001
    kd_pitch = .001
    ki_pitch = .00001

    kp_yaw = .00001
    kd_yaw = .001
    ki_yaw = .00001

    kp_lat = .001
    kd_lat = .2
    ki_lat = .0001

    kp_lon = .001
    kd_lon = .2
    ki_lon = .0001

    set_point = 150000
    set_latitude_point = 476418000
    set_longitude_point = -1221403000
    
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

        #set_altitude(actuation, actuation, actuation, actuation)

    def roll_oper(set_roll_angle):                            # for roll control

        global control_roll,exit_roll,err_roll,control_roll
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
        #set_altitude(actuation-control_roll, actuation+control_roll,actuation+control_roll, actuation-control_roll)
                                    
        if set_roll_angle != 0 :
            if (np.abs(err_roll) < .2):
                print "sucess--------------------roll------------------------------------------"
                exit_roll = 1
                cmd = 0


    def pitch_oper(set_pitch_angle):                          #for pitch control

        global control_pitch,exit_pitch,err_pitch,control_pitch
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
        #set_altitude(actuation+control_pitch, actuation-control_pitch,actuation+control_pitch, actuation-control_pitch)
                                    
        if set_pitch_angle != 0 :
            if (np.abs(err_pitch) < .2):
                print "sucess-----------------pitch---------------------------------------------"
                exit_pitch = 1
                cmd = 0

    def yaw_oper(set_yaw_angle):                          #for yaw control

        global control_yaw,exit_yaw,err_yaw,control_yaw
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
        #set_altitude(actuation+control_yaw, actuation+control_yaw,actuation-control_yaw, actuation-control_yaw)
                                    
        if set_yaw_angle != 0 :
            if (np.abs(err_yaw) < .2):
                print "sucess-----------------yaw---------------------------------------------"
                exit_yaw = 1
                cmd = 0
    def latitude_oper(set_latitude):   

        global enter_lat,err_latitude,pitch_desired

        err_latitude = latitude - set_latitude
        if np.abs(err_latitude) > 10 :

            enter_lat = 1
            que_lat.append(err_latitude)
            if len(que_lat) > 20:
              del que_lat[0]
            sum_lat = reduce(lambda s, v: s+v, que_lat)
            pitch_desired = kp_lat * err_latitude + kd_lat*(err_latitude-que_lat[-2])+ki_lat*sum_lat
            pitch_oper(pitch_desired)

        if np.abs(err_latitude) < 10 :
            enter_lat = 0
            pitch_oper(0)

    def longitude_oper(set_longitude):   

        global enter_lon,err_longitude,roll_desired

        err_longitude = set_longitude - longitude
        if np.abs(err_longitude) > 10 :

            enter_lon = 1
            que_lon.append(err_longitude)
            if len(que_lon) > 20:
              del que_lon[0]
            sum_lon = reduce(lambda s, v: s+v, que_lon)
            roll_desired = kp_lon * err_longitude + kd_lon*(err_longitude-que_lon[-2])+ki_lon*sum_lon
            roll_oper(roll_desired)

        if np.abs(err_longitude) < 10 :
            enter_lon = 0
            roll_oper(0)

    while not rospy.is_shutdown():
        
        altitude_lock(set_point) 
        count += 1
        
        if cmd == 1:

            exit_roll = 0
            roll_oper(5)
        
        #if (exit_roll ==1) | (cmd == 0):

         #   roll_oper(0)
            

        if cmd == 2:

            exit_pitch = 0
            pitch_oper(5)
        
        #if (exit_pitch ==1) | (cmd == 0):

            #pitch_oper(0)
            

        if cmd == 3:

            exit_yaw = 0
            yaw_oper(5)
        
        if (exit_yaw ==1) | (cmd == 0):

            yaw_oper(0)

        
        latitude_oper(set_latitude_point)

        longitude_oper(set_longitude_point)

    
          
        speed_motor_1 = actuation-control_roll+control_yaw+control_pitch
        speed_motor_2 = actuation+control_roll+control_yaw-control_pitch
        speed_motor_3 = actuation+control_roll-control_yaw+control_pitch
        speed_motor_4 = actuation-control_roll-control_yaw-control_pitch

        set_propeller(speed_motor_1,speed_motor_2,speed_motor_3,speed_motor_4)

        print "count: %s" % count
        print "Altitude:%s" % altitude
        print "yaw: %s" % yaw
        print "pitch: %s" % pitch
        print "roll: %s" % roll
        print "roll desired : %s" %roll_desired
        print "pitch desired : %s" %pitch_desired
        #print "cmd: %s" % cmd
        print "err-longitude: %s" % err_longitude
        print "err-latitud: %s" % err_latitude
        #print "enter-lat: %s" % enter_lat
       

        
       