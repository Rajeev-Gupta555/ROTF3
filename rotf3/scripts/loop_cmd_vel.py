import rospy, math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point

curr_msg = Twist()
omni_radii=0.2
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pubo = rospy.Publisher(
        '/rotf3/one_joint_velocity_controller/command', Float64, queue_size=1)
pubt = rospy.Publisher(
        '/rotf3/two_joint_velocity_controller/command', Float64, queue_size=1)
pubr = rospy.Publisher(
        '/rotf3/three_joint_velocity_controller/command', Float64, queue_size=1)

def get_msg(msg):
    global curr_msg
    curr_msg = msg

def loop():
    while(1):
        pub.publish(curr_msg)
        m1 = -curr_msg.angular.z*omni_radii - curr_msg.linear.y
        m2 = -curr_msg.angular.z*omni_radii + curr_msg.linear.y/2 + curr_msg.linear.x * math.sqrt(3)/2
        m3 = -curr_msg.angular.z*omni_radii + curr_msg.linear.y/2 - curr_msg.linear.x * math.sqrt(3)/2
        pubo.publish(-m1)
        pubt.publish(-m2)
        pubr.publish(-m3)
        rate.sleep()

rospy.Subscriber("loop_node", Twist, get_msg)

if __name__=='__main__':
    try:
        rospy.init_node("loop_node")
        rate = rospy.Rate(20)
        loop()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("F... Thank you")


