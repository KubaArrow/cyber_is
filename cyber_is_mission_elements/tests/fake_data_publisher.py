#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance

class FakeMissionPublisher:
    def __init__(self):
        rospy.init_node('fake_data_publisher')

        self.start_pub = rospy.Publisher('/start_collection', Bool, queue_size=1, latch=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)

        self.rate = rospy.Rate(1)  # 1 Hz
        self.steps = rospy.get_param('~steps', 30)  # liczba cykli symulacji
        self.current_step = 0

        self.x, self.y = 0.0, 0.0
        self.charge = 100.0  # Wh

        rospy.sleep(1.0)  # daj czas na podłączenie subów
        self.start_pub.publish(Bool(data=True))
        rospy.loginfo("Fake data publishing started.")

    def publish(self):
        while not rospy.is_shutdown() and self.current_step < self.steps:
            self.publish_odom()
            self.publish_battery()
            self.rate.sleep()
            self.current_step += 1

        # STOP
        self.start_pub.publish(Bool(data=False))
        rospy.loginfo("Fake data publishing stopped after %d steps.", self.steps)

    def publish_odom(self):
        dx = random.uniform(0.05, 0.15)
        dy = random.uniform(-0.05, 0.05)
        self.x += dx
        self.y += dy

        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.w = 1.0  # brak obrotu
        msg.twist.twist.linear.x = dx
        self.odom_pub.publish(msg)

    def publish_battery(self):
        usage = random.uniform(0.05, 0.3)  # symulacja zużycia
        self.charge = max(0.0, self.charge - usage)

        msg = BatteryState()
        msg.header.stamp = rospy.Time.now()
        msg.charge = self.charge
        self.battery_pub.publish(msg)

if __name__ == '__main__':
    try:
        publisher = FakeMissionPublisher()
        publisher.publish()
    except rospy.ROSInterruptException:
        pass
