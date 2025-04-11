#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8MultiArray

# Układ LED-ów zgodnie z rysunkiem (indeksy jako stringi)
led_layout = [
    [" ", " ", " ", "","","5", "6",  "7",  "8"," ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" ", "4", " ", " ", " ", " ", " ", " ", " ", " ", " ", "9", " "],
    [" ", "3", " ", " ", " ", " ", " ", " ", " ", " ", " ", "10", " "],
    [" ", "2", " ", " ", " ", " ", " ", " ", " ", " ", " ", "11", " "],
    [" ", "1", " ", " ", " ", " ", " ", " ", " ", " ", " ", "12", " "],
    [" ", "0", " ", " ", " ", " ", " ",  " ", " ", " ", " ","13", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " "],

    [ " ", " ", " ", "17", "16", " "," ", " ", " ", "15", "14"],
]

def display_leds_callback(msg: UInt8MultiArray):
    print("\033c", end="")  # clear screen

    num_leds = 18
    if len(msg.data) < num_leds * 4:
        rospy.logwarn("Oczekiwano %d bajtów, otrzymano %d", num_leds * 4, len(msg.data))
        return

    # Wyciągnij kolory (RGB, pomijamy Alpha)
    colors = []
    for i in range(num_leds):
        base = i * 4
        r = msg.data[base]
        g = msg.data[base + 1]
        b = msg.data[base + 2]
        colors.append((r, g, b))

    # Wyświetl zgodnie z układem
    for row in led_layout:
        line = ""
        for cell in row:
            if cell.strip().isdigit():
                index = int(cell)
                r, g, b = colors[index]
                line += f"\033[48;2;{r};{g};{b}m  \033[0m"  # kolorowy blok z odstępem
            else:
                line += "   "  # pusty blok (3 spacje dla wyrównania)
        print(line)

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
