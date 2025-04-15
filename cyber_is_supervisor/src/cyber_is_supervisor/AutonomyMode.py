import rospy
import subprocess
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
import time


class AutonomyMode:

    def __init__(self):
        self.state = ""
        self.process = None
        self.robot_state_sub = rospy.Subscriber("/robot_state", String, self.callback)
        self.state_publisher = rospy.Publisher("/robot_state", String, queue_size=10)
        self.leds_publisher = rospy.Publisher("/leds_mode", String, queue_size=10)
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.running = False
        self.navigation_process = None
        self.filters_process = None
        self.monitor_navigation_thread = None
        self.shutdown_requested = False

    def callback(self, data):
        if self.running:
            self.state = data.data
            self.state_switch()

    def state_switch(self):
        if self.state == "ABORT_MISSION":
            self.abort_mission()
        elif self.state == "START_MISSION":
            self.start_mission()
        elif self.state == "RESTART_MISSION":
            self.start_mode()



    def start_mode(self):
        rospy.loginfo("Starting autonomy mode...")
        self.prepare_mission()
        self.running = True


    def stop_mode(self):
        self.destroy_mission()
        self.state_publisher.publish("STOP_MANUAL_MODE")

    def prepare_mission(self):
        self.leds_publisher.publish("FRONT_MIN")
        self.leds_publisher.publish("SIDE_BLUE_STROBE")
        self.navigation_process = subprocess.Popen(['roslaunch', 'cyber_is_navigation', 'start_navigation.launch'])
        self.monitor_navigation_thread = threading.Thread(target=self.monitor_navigation_process)
        self.monitor_navigation_thread.daemon = True
        self.monitor_navigation_thread.start()
        self.filters_process = None
        self.leds_publisher.publish("SIDE_GREEN")
        self.state_publisher.publish("AUTONOMY_READY")
        rospy.loginfo("Prepared mission")

    def destroy_mission(self):
        self.running = False
        self.shutdown_requested = True
        if self.navigation_process and self.navigation_process.poll() is None:
            rospy.loginfo("Stopping autonomy mode...")
            self.navigation_process.terminate()
            self.navigation_process.wait()
        rospy.loginfo("Navigation destroyed...")

    def abort_mission(self):
        self.leds_publisher.publish("SIDE_RED_STROBE")
        self.state_publisher.publish("MISSION_FAILED")
        rospy.loginfo("Aborting mission...")
        self.destroy_mission()
        self.cmd_publisher.publish(Twist())
        self.state_publisher.publish("ROBOT_STOPPED")
        self.state_publisher.publish("WAITING_FOR_START")
        self.leds_publisher.publish("SIDE_GREEN_BREATH")

    def start_mission(self):
        rospy.loginfo("Starting mission...")

        pass

    def monitor_navigation_process(self):
        while self.running:
            time.sleep(2)
            if self.navigation_process and self.navigation_process.poll() is not None:
                if self.shutdown_requested:
                    rospy.loginfo("Navigation  terminated by user request.")
                else:
                    rospy.logerr("Navigation  process crashed.")
                    self.state_publisher.publish("ERROR_AUTONOMY_MODE")

                    # restart only if it wasn't shut down manually
                    time.sleep(3)
                    rospy.loginfo("Restarting  ...")
                    self.start_mode()
                self.running = False
                break
