#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist, Point
vx=0
vy=0
yaw = 0
angle = 0
MAX_VEL = 0.255
cmd = Twist()
vel = 0.1

def update_yaw(pmsg):
    global angle, yaw
    yaw = pmsg.z
    angle-=yaw*180/math.pi

pub = rospy.Publisher('loop_node', Twist, queue_size=10)
rospy.Subscriber("omni_posn", Point, update_yaw)

def reset():
    global angle, yaw, cmd, vx, vy
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0
    vx=0
    vy=0
    yaw = 0
    angle = 0

def teleop():
    rospy.loginfo("Enter angle (in degrees) to move || any letter to stop!!:")
    try:
        global vel, angle
        i = input()
        if i=='q':
            vel+=0.01 if vel+0.01<=MAX_VEL else 0.1
            print("vel set to", vel)
        elif i=='x':
            vel*=0.9
            print("vel set to", vel)
        elif i=='r':
            cmd.linear.x = cmd.linear.y = 0
            cmd.angular.z = 0.2
        elif i=='R':
            cmd.linear.x = cmd.linear.y = 0
            cmd.angular.z = -0.2
        else:
            angle = int(i)*math.pi/180
        vx = vel*math.cos(angle)
        vy = vel*math.sin(angle)
        cmd.linear.x = vx
        cmd.linear.y = vy
        # cmd.angular.z = yaw
    except ValueError:
        reset()
    pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("teleop_rotf3")
    while(1):
        teleop()
    rospy.spin()
