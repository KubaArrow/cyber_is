#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time

# Długość okna danych w sekundach
WINDOW_SIZE = 10

# Bufory na dane (czas, prędkość liniowa i kątowa)
time_buffer = deque()
linear_buffer = deque()
angular_buffer = deque()

start_time = None

def callback(msg):
    global start_time
    if start_time is None:
        start_time = time.time()
    t = time.time() - start_time

    # Dodaj nowe dane
    time_buffer.append(t)
    linear_buffer.append(msg.linear.x)
    angular_buffer.append(msg.angular.z)

    # Usuń stare dane, by zachować okno czasowe
    while time_buffer and (t - time_buffer[0] > WINDOW_SIZE):
        time_buffer.popleft()
        linear_buffer.popleft()
        angular_buffer.popleft()

def plot_callback(frame):
    if not time_buffer:
        return

    ax1.clear()
    ax2.clear()

    ax1.plot(time_buffer, linear_buffer, label='Linear Velocity (x)', color='blue')
    ax2.plot(time_buffer, angular_buffer, label='Angular Velocity (z)', color='red')

    ax1.set_ylabel("Linear Velocity [m/s]")
    ax2.set_ylabel("Angular Velocity [rad/s]")
    ax2.set_xlabel("Time [s]")

    ax1.legend()
    ax2.legend()
    ax1.grid()
    ax2.grid()

# Inicjalizacja ROS i subskrypcji
rospy.init_node('cmd_vel_plotter', anonymous=True)
rospy.Subscriber('/cmd_vel', Twist, callback)

# Przygotowanie wykresu
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
ani = FuncAnimation(fig, plot_callback, interval=100)

plt.tight_layout()
plt.show()
