import rospy
import subprocess
import threading
import time
from std_msgs.msg import String

class ManualMode:

    def __init__(self):
        self.process = None
        self.publisher = rospy.Publisher("/robot_state", String, queue_size=10, latch=True)
        self.leds_publisher = rospy.Publisher("/leds_mode", String, queue_size=10)
        self.monitor_thread = None
        self.running = False
        self.shutdown_requested = False  # <-- dodajemy flagę

    def start_mode(self):
        if not self.process or self.process.poll() is not None:
            rospy.loginfo("Starting manual mode...")
            self.publisher.publish("START_MANUAL_MODE")
            self.leds_publisher.publish("FRONT_WHITE_25")
            self.leds_publisher.publish("SIDE_GREEN_BREATH")
            self.shutdown_requested = False  # <-- resetujemy flagę
            self.process = subprocess.Popen(
                ['roslaunch', 'cyber_is_manual_controller', 'start_manual_mode.launch']
            )
            # Start monitoring in a background thread
            self.running = True

            rospy.sleep(7)
            self.leds_publisher.publish("FRONT_WHITE_100")
            self.leds_publisher.publish("SIDE_GREEN_75")
            self.publisher.publish("READY_MANUAL_MODE")

    def stop_mode(self):
        self.running = False
        self.shutdown_requested = True  # <-- ustawiamy flagę
        if self.process and self.process.poll() is None:
            rospy.loginfo("Stopping manual mode...")
            self.process.terminate()
            self.process.wait()
            self.publisher.publish("STOP_MANUAL_MODE")


