import ctypes
import rospy, math
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Point, Twist

class PID(object):
   def __init__(self, name, kp, kd, ki, minOutput, maxOutput, integratorMin, integratorMax):
      self.name = name
      self.m_kp = kp                       
      self.m_kd = kd                       
      self.m_ki = ki                       
      self.m_minOutput = minOutput         
      self.m_maxOutput = maxOutput         
      self.m_integratorMin = integratorMin  

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

      time = float(rospy.Time.to_sec(rospy.Time.now()))

      dt = time - self.m_previousTime

      error = targetValue - value

      self.m_integral += error * dt

      self.m_integral = max(min(self.m_integral, self.m_integratorMax), self.m_integratorMin)

      p = self.m_kp * error

      d = 0
      if dt > 0:
         d = self.m_kd * (error - self.m_previousError) / dt

      i = self.m_ki * self.m_integral

      output = p + d + i
      self.m_previousError = error
      self.m_previousTime = time

      return max(min(output, self.m_maxOutput), self.m_minOutput)
