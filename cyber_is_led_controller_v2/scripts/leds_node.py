#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8MultiArray, String
from cyber_is_led_controller.leds_controller import LedsController

ctrl = LedsController()

def callback(msg: String):
    ctrl.set_mode(msg.data)
    rospy.loginfo(f"[Subscriber] Received mode: {msg.data}")

def main():
    rospy.init_node("leds_controller", anonymous=True)

    leds_topic  = rospy.get_param("/led_controller/leds_topic",   "/leds")
    state_topic = rospy.get_param("/led_controller/state_topic", "/leds_mode")
    freq        = rospy.get_param("/led_controller/frequency",   12)

    leds_pub  = rospy.Publisher(leds_topic, UInt8MultiArray, queue_size=10)
    rospy.Subscriber(state_topic, String, callback)

    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        ctrl.tick()

        msg = UInt8MultiArray()
        msg.data = ctrl.get_leds()
        leds_pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
