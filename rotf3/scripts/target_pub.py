#!/usr/bin/env python
import rospy, math
from std_msgs.msg import Float32MultiArray

def talker():
    rospy.init_node('omini', anonymous=True)
    pub = rospy.Publisher('pc_to_rasp_chatter', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    my_data = []
    while not rospy.is_shutdown():
        array = Float32MultiArray()
        try:
            my_data = [float(x) for x in input().split()]           #coordinates x, y, angle
            array.data = my_data
            pub.publish(array)
            print(my_data)
            rospy.loginfo("sending data: x=%f, y=%f, angle=%f w.r.t Gnd Frame", my_data[0], my_data[1], my_data[2])
        except (IndexError, ValueError, KeyboardInterrupt):
            array.data = [0.0]
            pub.publish(array)
            rospy.loginfo("unexpected input, stopping bot!!!")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
