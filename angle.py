#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def send_servo_commands():
    pub = rospy.Publisher('/servo_commands', Float32MultiArray, queue_size=10)
    rospy.init_node('servo_controller', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        angles = [0.0, 45, 90.2, 135.8, 160.4, 180]  # Example float angles
        msg = Float32MultiArray()
        msg.data = angles
        pub.publish(msg)
        rospy.loginfo(f"Sent angles: {angles}")
        rate.sleep()

if __name__ == '__main__':
    try:
        send_servo_commands()
    except rospy.ROSInterruptException:
        pass
