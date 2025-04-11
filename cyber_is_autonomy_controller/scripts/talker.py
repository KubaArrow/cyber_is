#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/auto', String, queue_size=10)
    rospy.init_node('auto_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz (1 message per second)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "hello world"
        rospy.loginfo(f"Publishing: {msg.data}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
