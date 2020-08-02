#!/usr/bin/env python
import rospy
import operator
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16,Float64,Int8,Bool,Float32
from PID import *
import numpy as np
from std_msgs.msg import Float64,UInt8,Int64


x=0
y=2
tickgreen=1
twist = Twist()
twist2 = Twist() 
twist3 = Twist()
joycontrol=0
speed=0
#feedback = 50.0
outputpid = 0.0
mode = 1
stage = 0
mission = 0
kp = 0.01
ki = 0.00
kd = 0.00
pid = PID()
run = 0
brake = 0
emer = 1


class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/angle',Float64,self.cbFollowLane, queue_size = 10)
        self.sub_light = rospy.Subscriber('/light',Int8,self.get_light,queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/Donkey/Arduino/cmd_vel',Twist, queue_size = 1)
        self.pub_emer = rospy.Publisher('/Donkey/Arduino/emergency',Bool, queue_size = 1)
        rospy.on_shutdown(self.fnShutDown)
    def get_light(self,data):
        self.light = data.data
    def cbFollowLane(self,data):
        global twist
        global pid
        global tickgreen
        global stage
        global kp
        global ki
        global kd
        global run
        global brake
        global emer
        emer = 1
        self.pub_emer.publish(emer)
        kp = 1
        ki = 0
        kd = 0
        li = 0
        lc = 0
        pv_angle_raw = data.data
        #pv_angle = (pv_angle_raw * 1.42857) - 42.857
        pid.SetPoint = 50.0
        pid.update(pv_angle_raw)
        outputpid = pid.output
        #pid = PID(kp,ki,kd)
        twist = Twist()
        if self.light == 1:
            run = 1
        if run == 0:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        if run == 1:
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            if pv_angle_raw > 85:
                li = 0.65
            if pv_angle_raw < 15:
                #az = 0.5
                li = 0.65
            if pv_angle_raw <=85 and pv_angle_raw >=15:
                #az = outputpid* 1 / 100
                li = 0.65 
            az = outputpid* 1 / 100
            twist.linear.x = li
            twist.angular.z = az
            self.pub_cmd_vel.publish(twist)
            print(outputpid)
            pid = PID(kp,ki,kd)



    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)


    def main(self):
        #global kp
        rospy.spin()
        
        #pid = PID(kp,0,0)
        #pid.SetPoint=50.0
        #pid.setSampleTime(0.01)
        
        

if __name__ == '__main__':
    #global kp
    rospy.init_node('control_lane')
    node = ControlLane()
    #pid = PID(kp,0,0)
    pid.SetPoint=50.0
    pid.setSampleTime(0.01)
    node.main()


