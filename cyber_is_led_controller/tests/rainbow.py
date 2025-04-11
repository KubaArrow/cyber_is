#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8MultiArray
import colorsys

NUM_LEDS = 18
FPS = 30              # liczba klatek na sekundę
ANIMATION_SPEED = 0.005  # przyrost wartości hue w każdej iteracji

def main():
    rospy.init_node("rainbow_animator", anonymous=True)
    leds_pub = rospy.Publisher("/leds", UInt8MultiArray, queue_size=10)
    rospy.loginfo("Rainbow Animator uruchomiony.")

    rate = rospy.Rate(FPS)
    base_hue = 0.0

    while not rospy.is_shutdown():
        msg = UInt8MultiArray()
        data = []
        for i in range(NUM_LEDS):
            # Obliczamy hue dla danego LED-a, rozłożone równomiernie
            hue = (base_hue + (i / NUM_LEDS)) % 1.0
            # Konwersja z HSV (pełne nasycenie i jasność) do RGB
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            # Przeliczenie wartości do zakresu 0-255
            r_int = int(r * 255)
            g_int = int(g * 255)
            b_int = int(b * 255)
            a_int = 255  # pełna przezroczystość
            data.extend([r_int, g_int, b_int, a_int])
        msg.data = data

        leds_pub.publish(msg)

        # Przesuwamy bazowy hue dla animacji
        base_hue = (base_hue + ANIMATION_SPEED) % 1.0

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
