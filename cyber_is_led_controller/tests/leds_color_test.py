#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8MultiArray

def display_leds_callback(msg: UInt8MultiArray):
    # Wyczyść ekran konsoli
    print("\033c", end="")

    # Upewnij się, że mamy dane dla 12 LED-ów (4 bajty każdy)
    num_leds = 12
    if len(msg.data) < num_leds * 4:
        rospy.logwarn("Otrzymano zbyt mało danych, oczekiwano %d bajtów, otrzymano %d",
                      num_leds * 4, len(msg.data))
        return

    output = ""
    for i in range(num_leds):
        base = i * 4
        r = msg.data[base]
        g = msg.data[base + 1]
        b = msg.data[base + 2]
        # Opcjonalnie: a = msg.data[base + 3]
        # Używamy sekwencji ANSI do ustawienia tła na kolor RGB
        # Blok to dwa znaki spacji z tłem ustawionym na dany kolor
        output += f"\033[48;2;{r};{g};{b}m  \033[0m "
    print(output)

def main():
    rospy.init_node("led_visualizer", anonymous=True)
    rospy.Subscriber("/leds", UInt8MultiArray, display_leds_callback)
    rospy.loginfo("LED Visualizer node uruchomiony. Nasłuchiwanie na topicu /leds")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
