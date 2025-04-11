#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess

class ModeSupervisor:
    def __init__(self):
        rospy.init_node('mode_supervisor')

        self.manual_process = None
        self.auto_process = None

        # Start with manual mode
        self.start_manual_mode()

        rospy.Subscriber('/robot_mode', String, self.mode_callback)
        rospy.loginfo("Mode Supervisor started and listening to /robot_mode...")
        rospy.spin()

    def start_manual_mode(self):
        self.stop_autonomy_mode()
        if not self.manual_process or self.manual_process.poll() is not None:
            rospy.loginfo("Starting manual mode...")
            self.manual_process = subprocess.Popen(
                ['roslaunch', 'cyber_is_manual_controller', 'start_manual_mode.launch']
            )

    def stop_manual_mode(self):
        if self.manual_process and self.manual_process.poll() is None:
            rospy.loginfo("Stopping manual mode...")
            self.manual_process.terminate()
            self.manual_process.wait()

    def start_autonomy_mode(self):
        self.stop_manual_mode()
        if not self.auto_process or self.auto_process.poll() is not None:
            rospy.loginfo("Starting autonomy mode...")
            self.auto_process = subprocess.Popen(
                ['roslaunch', 'cyber_is_autonomy_controller', 'start_autonomy_mode.launch']
            )

    def stop_autonomy_mode(self):
        if self.auto_process and self.auto_process.poll() is None:
            rospy.loginfo("Stopping autonomy mode...")
            self.auto_process.terminate()
            self.auto_process.wait()

    def mode_callback(self, msg):
        mode = msg.data.strip().upper()
        if mode == "MANUAL":
            self.start_manual_mode()
        elif mode == "AUTO":
            self.start_autonomy_mode()
        else:
            rospy.logwarn(f"Unknown mode received: {mode}")

if __name__ == '__main__':
    try:
        ModeSupervisor()
    except rospy.ROSInterruptException:
        pass
