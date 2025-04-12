#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool

class SearchStartNodeTester:
    def __init__(self):
        rospy.init_node('search_start_tester')

        self.robot_state_topic = rospy.get_param('~robot_state_topic', '/robot_state')
        self.line_detector_topic = rospy.get_param('~line_detector_topic', '/line_detector_position')
        self.magnet_sensor_topic = rospy.get_param('~magnet_sensor_topic', '/magnet_filtered')

        self.state_pub = rospy.Publisher(self.robot_state_topic, String, queue_size=1, latch=True)
        self.line_pub = rospy.Publisher(self.line_detector_topic, String, queue_size=1, latch=True)
        self.magnet_pub = rospy.Publisher(self.magnet_sensor_topic, Bool, queue_size=1, latch=True)

        rospy.sleep(1.0)  # poczekaj na subskrybentów

        self.run_test()

    def run_test(self):
        rospy.loginfo("Step 1: Sending SEARCH_START")
        self.state_pub.publish(String(data="SEARCH_START"))

        rospy.Timer(rospy.Duration(10.0), self.send_full_line, oneshot=True)
        rospy.Timer(rospy.Duration(15.0), self.send_magnet_true, oneshot=True)

    def send_full_line(self, _):
        rospy.loginfo("Step 2: Sending FULL_LINE")
        self.line_pub.publish(String(data="FULL_LINE"))

    def send_magnet_true(self, _):
        rospy.loginfo("Step 3: Sending magnet TRUE")
        self.magnet_pub.publish(Bool(data=True))

if __name__ == '__main__':
    try:
        SearchStartNodeTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

