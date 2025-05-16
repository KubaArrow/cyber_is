#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import UInt16MultiArray

def publisher():
    rospy.init_node('line_generator')
    pub = rospy.Publisher('/line_detector', UInt16MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = UInt16MultiArray()
        #msg.data = [random.randint(0, 1000) for _ in range(5)]
        msg.data = [2200,3100,0,3100,2800]
        rospy.loginfo(f"Publishing: {msg.data}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
