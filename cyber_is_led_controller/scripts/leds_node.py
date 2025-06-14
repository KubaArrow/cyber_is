#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import String
from cyber_is_led_controller.leds_modes import Modes

modes = Modes()

def callback(msg):
    modes.set_mode(msg.data)
    rospy.loginfo(f"[Subscriber] Received message: {msg.data}")

def main():

    rospy.init_node("leds_controller", anonymous=True)

    leds_topic = rospy.get_param("/led_controller/leds_topic", "/leds")
    freq = rospy.get_param("/led_controller/frequency", 12)
    state_topic = rospy.get_param("/led_controller/leds_topic", "/leds_mode")

    leds_pub = rospy.Publisher(leds_topic, UInt8MultiArray, queue_size=10)
    leds_mode_sub=rospy.Subscriber(state_topic, String, callback)
    
    rate = rospy.Rate(freq)

    while not rospy.is_shutdown():
        msg = UInt8MultiArray()
        msg.data = modes.get_leds()
        leds_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass