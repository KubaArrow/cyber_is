#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import subprocess

from cyber_is_supervisor.manual_mode import ManualMode

from cyber_is_supervisor.autonomy_mode import AutonomyMode


class ModeSupervisor:
    def __init__(self):
        rospy.init_node('mode_supervisor')

        self.current_mode = "MANUAL"
        self.manual = ManualMode()
        self.auto = AutonomyMode()

        # Start with manual mode
        self.manual.start_mode()

        rospy.Subscriber('/robot_mode', String, self.mode_callback)
        rospy.loginfo("Mode Supervisor started and listening to /robot_mode...")
        rospy.spin()

    def mode_callback(self, msg):
        mode = msg.data.strip().upper()
        if mode == "MANUAL"  and not self.manual.running:
            self.auto.stop_mode()
            self.manual.start_mode()
        elif mode == "AUTO" and not self.auto.running:
            self.manual.stop_mode()
            self.auto.start_mode()
        else:
            rospy.logwarn(f"Unknown mode received: {mode}")

if __name__ == '__main__':
    try:
        ModeSupervisor()
    except rospy.ROSInterruptException:
        pass
