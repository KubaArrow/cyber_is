#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import ColorRGBA

NUM_LEDS = 12
current_color = [255, 255, 0, 120]

def change_color_callback(msg: ColorRGBA):
    global current_color
    def convert(value):
        return int(value * 255) if value <= 1.0 else int(value)
    r = convert(msg.r)
    g = convert(msg.g)
    b = convert(msg.b)
    a = convert(msg.a)
    current_color = [r, g, b, a]
    rospy.loginfo(f"Zmiana koloru LED-ów na: R={r}, G={g}, B={b}, A={a}")

def main():
    rospy.init_node('led_controller', anonymous=True)
    leds_pub = rospy.Publisher('/leds', UInt8MultiArray, queue_size=10)
    rospy.Subscriber('/change_leds', ColorRGBA, change_color_callback)

    rospy.loginfo("Node led_controller uruchomiony")

    rospy.sleep(1)  # Czekamy, aż publisher się zarejestruje

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        msg = UInt8MultiArray()
        data = []
        for _ in range(NUM_LEDS):
            data.extend(current_color)
        msg.data = data

        leds_pub.publish(msg)
        rospy.loginfo("Wysłano wiadomość na /leds")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
