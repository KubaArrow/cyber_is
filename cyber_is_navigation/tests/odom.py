#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time

# Czas trwania okna danych [s]
WINDOW_SIZE = 10

# Bufory na dane
time_buffer = deque()
linear_buffer = deque()
angular_buffer = deque()

start_time = None

def odom_callback(msg):
    global start_time
    if start_time is None:
        start_time = time.time()
    t = time.time() - start_time

    linear_x = msg.twist.twist.linear.x
    angular_z = msg.twist.twist.angular.z

    time_buffer.append(t)
    linear_buffer.append(linear_x)
    angular_buffer.append(angular_z)

    # Usuń dane starsze niż okno czasowe
    while time_buffer and (t - time_buffer[0] > WINDOW_SIZE):
        time_buffer.popleft()
        linear_buffer.popleft()
        angular_buffer.popleft()

def update_plot(frame):
    if not time_buffer:
        return

    ax1.clear()
    ax2.clear()

    ax1.plot(time_buffer, linear_buffer, label='Linear Velocity (x)', color='green')
    ax2.plot(time_buffer, angular_buffer, label='Angular Velocity (z)', color='orange')

    ax1.set_ylabel("Linear Velocity [m/s]")
    ax2.set_ylabel("Angular Velocity [rad/s]")
    ax2.set_xlabel("Time [s]")

    ax1.legend()
    ax2.legend()
    ax1.grid()
    ax2.grid()

# Inicjalizacja ROS i subskrybenta
rospy.init_node('odom_plotter', anonymous=True)
rospy.Subscriber('/low_level_odom', Odometry, odom_callback)

# Inicjalizacja wykresu
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
ani = FuncAnimation(fig, update_plot, interval=100)

plt.tight_layout()
plt.show()
