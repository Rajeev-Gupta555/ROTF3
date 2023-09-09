#!/usr/bin/env python
import math
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class PID(object):
    def __init__(self, name, kp, kd, ki, minOutput, maxOutput, integratorMin, integratorMax):
        self.name = name
        self.m_kp = kp                       
        self.m_kd = kd                       
        self.m_ki = ki                       
        self.m_minOutput = minOutput         
        self.m_maxOutput = maxOutput         
        self.m_integratorMin = integratorMin  
        self.m_integratorMax = integratorMax
        self.m_integral = 0
        self.m_previousError = 0 
        self.m_previousTime = float(rospy.Time.to_sec(rospy.Time.now()))

    def reset (self):
        self.m_integral = 0
        self.m_previousError = 0
        self.m_previousTime = float(rospy.Time.to_sec(rospy.Time.now()))

    def setIntegral (self, integral):
        self.m_integral = integral

    def set_ki (self):
        return self.m_ki

    def update (self, value, targetValue):
        if abs(targetValue) <= 0.001:
            return 0
        time = float(rospy.Time.to_sec(rospy.Time.now()))
        dt = time - self.m_previousTime
        error = targetValue - value
        self.m_integral += error * dt
        self.m_integral = max(min(self.m_integral, self.m_integratorMax), self.m_integratorMin)
        p = self.m_kp * targetValue
        d = 0
        if dt > 0:
            d = self.m_kd * (error - self.m_previousError) / dt
        i = self.m_ki * self.m_integral
        output = p + d + i
        self.m_previousError = error
        self.m_previousTime = time
        # rospy.loginfo("name:%s, error:%f, p:%f, output:%f", self.name, error, p, output)
        return max(min(output, self.m_maxOutput), self.m_minOutput)

class Get_Odom():
    def __init__(self):
        self.last_time = rospy.Time.now()
        # a bool var to initiate both encoder posn(past and now) to the same values
        self.tag = True
        self.omni_radii = 0.2
        self.posn = Point()
        self.encode_data_past = [0.0, 0.0, 0.0]
        self.encode_data_now = [0.0, 0.0, 0.0]
        # omni vel wrt its own frame 
        self.vel = Twist()

        self.motor_cmd_vel = Twist()
        self.motor_curr_vel = Twist()
        self.odom = Odometry()
        self.posn.x = 0
        self.posn.y = 0
        self.posn.z = 0
        self.joint = ["joint1", "joint2", "joint3"]
        rospy.Subscriber("cmd_vel", Twist, self._send_vel)
        rospy.Subscriber("omni_to_rasp_chatter", Point, self._update_crnt_pose)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.vel_pub = rospy.Publisher("rasp_to_omni_chatter", Twist, queue_size=50)
        self.posn_pub = rospy.Publisher("omni_posn", Point, queue_size=50)
        # for physical units
        self.max_vel_theta = rospy.get_param("/move_base/DWAPlannerROS/max_vel_theta", 0.57)
        self.m_pid1 = PID(self.joint[0],1   ,0, 0, -0.255, 0.255, -0.225, 0.225)
        self.m_pid2 = PID(self.joint[1],1.2   ,0, 0, -0.255, 0.255, -0.225, 0.225)
        self.m_pid3 = PID(self.joint[2],1 ,0, 0, -0.255, 0.255, -0.225, 0.225)
        # for motor units
        # self.m_pid1 = PID(self.joint[0],1 , 1, 1, -255, 255, -225, 225)
        # self.m_pid2 = PID(self.joint[1],1 , 1, 1, -255, 255, -225, 225)
        # self.m_pid3 = PID(self.joint[2],0.9 , 1, 1, -255, 255, -225, 225)

    def _update_crnt_pose(self, pmsg):
        try:
            # dt = 0.02
            dt =  (rospy.Time.now() - self.last_time).to_sec()
            self.last_time = rospy.Time.now()
            # getting encoder data
            self.encode_data_now[2] = pmsg.z
            self.encode_data_now[1] = pmsg.y
            self.encode_data_now[0] = pmsg.x
            if self.tag==True: 
                for i in range(3):
                    self.encode_data_past[i] = self.encode_data_now[i]
                self.tag = False
            # calculation velocities in robot's frame
            if dt==0:
                dt=0.02
            self.vel.linear.x = (self.encode_data_now[0]-self.encode_data_past[0])/dt
            self.vel.linear.y = (self.encode_data_now[1]-self.encode_data_past[1])/dt
            self.vel.angular.z =  (self.encode_data_now[2]-self.encode_data_past[2])/dt
            # current vel of each motor
            self.motor_curr_vel.linear.x  = -self.vel.angular.z*self.omni_radii - self.vel.linear.y
            self.motor_curr_vel.linear.y  = -self.vel.angular.z*self.omni_radii + self.vel.linear.y/2 + self.vel.linear.x * math.sqrt(3)/2
            self.motor_curr_vel.linear.z  = -self.vel.angular.z*self.omni_radii + self.vel.linear.y/2 - self.vel.linear.x * math.sqrt(3)/2
            # self.motor_curr_vel.linear.x, self.motor_curr_vel.linear.y, self.motor_curr_vel.linear.z = self._translate_vel(self.motor_curr_vel.linear.x, self.motor_curr_vel.linear.y, self.motor_curr_vel.linear.z)

            # wrt grnd frame
            delta_y = (self.vel.linear.y * math.cos(self.posn.z) - self.vel.linear.x * math.sin(self.posn.z)) * dt
            delta_x = (self.vel.linear.y * math.sin(self.posn.z) + self.vel.linear.x * math.cos(self.posn.z)) * dt
            delta_th = self.vel.angular.z * dt
            # setting current posn
            self.posn.x = self.posn.x + delta_x
            self.posn.y = self.posn.y + delta_y
            self.posn.z = self.posn.z + delta_th
            self.posn_pub.publish(self.posn)
            # converting to odomentry()
            self.odom.header.stamp = self.last_time
            self.odom.header.frame_id = "odom"
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.posn.z)
            self.odom.pose.pose = Pose(Point(self.posn.x, self.posn.y, 0.), Quaternion(*odom_quat))
            self.odom.child_frame_id = "base_footprint"
            self.odom.twist.twist = Twist(Vector3(self.vel.linear.x, self.vel.linear.y, 0), Vector3(0, 0, self.vel.angular.z))
            # publish odom
            self.odom_pub.publish(self.odom)
            # send transform
            br = tf.TransformBroadcaster()
            br.sendTransform((self.posn.x, self.posn.y, 0),
                             tf.transformations.quaternion_from_euler(0, 0, self.posn.z),
                             rospy.Time.now(),
                             "base_footprint",
                             "odom")
            for i in range(3):
                self.encode_data_past[i] = self.encode_data_now[i]

        except (rospy.ROSInterruptException, AttributeError, TypeError):
            rospy.loginfo("An error occured, reseting values to zero...")
            self.posn.x = 0
            self.posn.y = 0
            self.posn.z = 0
            self.encode_data_past = [0.0, 0.0, 0.0]
            self.encode_data_now = [0.0, 0.0, 0.0]
    
    def _translate_vel(self, m1, m2, m3):
        m = [m1, m2, m3]
        # rospy.loginfo("motor physical value %f, %f, %f", m1, m2, m3)
        # fit calculated m* values from -255 to 255
        l = [abs(v) for v in m]
        maxx = max(max(l), 0.255)
        for i in range(3):
            m[i] = m[i]*225/maxx
        # directly input to motors m1, m2, m3
        return m[0], m[1], m[2]

    def _send_vel(self, msg):
        # calculate values for each motor (physical units)
        m1 = -msg.angular.z*self.omni_radii - msg.linear.y
        m2 = -msg.angular.z*self.omni_radii + msg.linear.y/2 + msg.linear.x * math.sqrt(3)/2
        m3 = -msg.angular.z*self.omni_radii + msg.linear.y/2 - msg.linear.x * math.sqrt(3)/2
        # rospy.loginfo("omni physical value %f, %f, %f", msg.linear.x, msg.linear.y, msg.angular.z)
        # rospy.loginfo("motor physical value %f, %f, %f", m1, m2, m3)
    
        # don't be confuse by motor_vel.linear.* 
        # values below are for directly input to motors m1, m2, m3
        # m1, m2, m3 = self._translate_vel(m1, m2, m3)
        self.motor_cmd_vel.linear.x = self.m_pid1.update(self.motor_curr_vel.linear.x, m1)
        self.motor_cmd_vel.linear.y = self.m_pid2.update(self.motor_curr_vel.linear.y, m2)
        self.motor_cmd_vel.linear.z = self.m_pid3.update(self.motor_curr_vel.linear.z, m3)
        rospy.loginfo("current vel.. %f, %f, %f, and \ncmd, %f, %f, %f", self.motor_curr_vel.linear.x, 
            self.motor_curr_vel.linear.y, 
            self.motor_curr_vel.linear.z, self.motor_cmd_vel.linear.x, 
            self.motor_cmd_vel.linear.y, 
            self.motor_cmd_vel.linear.z)
        # to remap values from -255 to 255
        self.motor_cmd_vel.linear.x, self.motor_cmd_vel.linear.y, self.motor_cmd_vel.linear.z = \
            self._translate_vel(self.motor_cmd_vel.linear.x, 
                                self.motor_cmd_vel.linear.y,
                                self.motor_cmd_vel.linear.z)

        self.vel_pub.publish(self.motor_cmd_vel)
        # rospy.loginfo("current posn x, y, th: %f, %f, %f", self.posn.x, self.posn.y,
        #                 self.posn.z)
        # rospy.loginfo("current motor vel, %f, %f, %f", self.motor_curr_vel.linear.x, 
        #     self.motor_curr_vel.linear.y, 
        #     self.motor_curr_vel.linear.z)
        # rospy.loginfo("cmd motor vel, %f, %f, %f", self.motor_cmd_vel.linear.x, 
        #     self.motor_cmd_vel.linear.y, 
        #     self.motor_cmd_vel.linear.z)



if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.loginfo("Node started")
    get_odom = Get_Odom()
    rospy.spin()