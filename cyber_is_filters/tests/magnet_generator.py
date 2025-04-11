#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import random

def magnet_generator():
    rospy.init_node('magnet_generator')
    pub = rospy.Publisher('/magnet', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = [random.randint(0, 1000) for _ in range(3)]  # np. wartości XYZ pola magnetycznego
        pub.publish(msg)
        rospy.loginfo(f"Published /magnet: {msg.data}")
        rate.sleep()

if __name__ == '__main__':
    try:
        magnet_generator()
    except rospy.ROSInterruptException:
        pass
