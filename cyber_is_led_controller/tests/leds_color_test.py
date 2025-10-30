#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8MultiArray

# Grid layout for ANSI preview
led_layout = [
    [" ", " ", " ", "",  "","9",  "10", "11", "12", "13", " ",  " ",  " ",  " "],
    [" ", "8",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",  "", " ",  "14", " "],
    [" ", "7",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",   ""," ",  "15", " "],
    [" ", "6",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",  "", " ",  "16", " "],
    [" ", "5",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",  "", " ",  "17", " "],
    [" ", "4",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",  "", " ",  "18", " "],
    [" ", "3",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",  "", " ",  "19", " "],
    [" ", "2",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",  "", " ",  "20", " "],
    [" ", "1",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",  "", " ",  "21", " "],
    [" ", "0",  " ", " ",  " ",  " ", " ",  " ",  " ",  " ",  "", " ",  "22", " "],
]

NUM_LEDS = 23  # 9 left + 6 front + 9 right


def display_leds_callback(msg: UInt8MultiArray):
    """Render LEDs in terminal using ANSI background colors."""
    print("\033c", end="")  # clear screen

    expected_bytes = NUM_LEDS * 4
    if len(msg.data) < expected_bytes:
        rospy.logwarn(
            "Oczekiwano %d bajtów, otrzymano %d", expected_bytes, len(msg.data)
        )
        return

    # Extract RGB tuples (skip Alpha channel)
    colors = [
        (msg.data[i * 4], msg.data[i * 4 + 1], msg.data[i * 4 + 2])
        for i in range(NUM_LEDS)
    ]

    # Print according to the layout
    for row in led_layout:
        line = ""
        for cell in row:
            if cell.strip().isdigit():
                idx = int(cell)
                r, g, b = colors[idx]
                line += f"\033[48;2;{r};{g};{b}m  \033[0m"
            else:
                line += "   "  # empty placeholder (3 spaces)
        print(line)


def main():
    rospy.init_node("led_visualizer", anonymous=True)
    rospy.Subscriber("/leds", UInt8MultiArray, display_leds_callback)
    rospy.loginfo(
        "LED Visualizer node uruchomiony. Nasłuchiwanie na topicu /leds (24 diody)"
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
