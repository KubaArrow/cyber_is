import rospy
import subprocess
import threading
import time
from std_msgs.msg import String

class ManualMode:

    def __init__(self):
        self.process = None
        self.publisher = rospy.Publisher("/robot_state", String, queue_size=10)
        self.leds_publisher = rospy.Publisher("/leds_mode", String, queue_size=10)
        self.monitor_thread = None
        self.running = False
        self.shutdown_requested = False  # <-- dodajemy flagę

    def start_mode(self):
        if not self.process or self.process.poll() is not None:
            rospy.loginfo("Starting manual mode...")
            self.publisher.publish("START_MANUAL_MODE")
            self.leds_publisher.publish("FRONT_HALF")
            self.leds_publisher.publish("SIDE_GREEN_STROBE")
            self.leds_publisher.publish("BACK_HALF")
            self.shutdown_requested = False  # <-- resetujemy flagę
            self.process = subprocess.Popen(
                ['roslaunch', 'cyber_is_manual_controller', 'start_manual_mode.launch']
            )
            # Start monitoring in a background thread
            self.running = True
            self.monitor_thread = threading.Thread(target=self.monitor_process)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            rospy.sleep(3)
            self.leds_publisher.publish("SIDE_GREEN")
            self.publisher.publish("READY_MANUAL_MODE")

    def stop_mode(self):
        self.running = False
        self.shutdown_requested = True  # <-- ustawiamy flagę
        if self.process and self.process.poll() is None:
            rospy.loginfo("Stopping manual mode...")
            self.process.terminate()
            self.process.wait()
            self.publisher.publish("STOP_MANUAL_MODE")

    def monitor_process(self):
        while self.running:
            time.sleep(2)
            if self.process and self.process.poll() is not None:
                if self.shutdown_requested:
                    rospy.loginfo("Manual mode terminated by user request.")
                else:
                    rospy.logerr("Manual mode process crashed.")
                    self.publisher.publish("ERROR_MANUAL_MODE")

                    # restart only if it wasn't shut down manually
                    time.sleep(3)
                    rospy.loginfo("Restarting manual mode...")
                    self.start_mode()
                self.running = False
                break
