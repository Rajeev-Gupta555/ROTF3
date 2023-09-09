import serial, rospy
from geometry_msgs.msg import Point

# past_posn_z = 0.0
def read_write(pub, point):
    global past_posn_z
    data = arduino.readline()
    if len(data)==0: return
    if(data[0]==36):
        # rospy.loginfo(data[1:-2])
        l = [float(x) for x in data[1:-2].split()]
        if len(l)==3: 
            point.x = l[0]
            point.y = l[1]
            # point.z = l[2]
            if l[2]-past_posn_z > 50:
                point.z -= 2
            elif l[2]-past_posn_z < -50:
                point.z += 2
            else:
                point.z += l[2]-past_posn_z
            pub.publish(point)
            past_posn_z = l[2]
            rospy.loginfo(l)
    else:
        rospy.loginfo(data)

if __name__ == "__main__":
    past_posn_z = 0.0
    rospy.init_node('read_from_arduino')
    rate=rospy.Rate(100)
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=5)
    point = Point()
    pub = rospy.Publisher("omni_to_rasp_chatter", Point, queue_size=10)
    while not rospy.is_shutdown():
        read_write(pub, point)
        rate.sleep()
