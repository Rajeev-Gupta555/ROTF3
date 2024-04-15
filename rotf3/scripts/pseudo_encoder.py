#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Point

def talker():
    rospy.init_node('pseudo_encoder', anonymous=True)
    pub = rospy.Publisher('omni_to_rasp_chatter', Point, queue_size=10)
    rate = rospy.Rate(50) # 10hz
    point = Point()
    while not rospy.is_shutdown():
        try:
            # point.x+=0.1
            # point.y+=0.1
            # point.z+=0.0
            pub.publish(point)
            rospy.loginfo("sending data: x=%f, y=%f, angle=%f w.r.t Gnd Frame", point.x, point.y, point.z)
        except (IndexError, ValueError, KeyboardInterrupt):
            rospy.loginfo("unexpected input, stopping bot!!!")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
