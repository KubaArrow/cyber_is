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
        self.state_publisher = rospy.Publisher("/robot_state", String, queue_size=10, latch=True)
        self.leds_publisher = rospy.Publisher("/leds_mode", String, queue_size=10)
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.running = False
        self.navigation_process = None
        self.filters_process = None
        self.mission_process = None
        self.colletor_process = None

    def callback(self, data):
        if self.running:
            self.state = data.data
            self.state_switch()

    def state_switch(self):
        if self.state == "ABORT_MISSION":
            self.abort_mission()
        elif self.state == "RESTART_MISSION":
            self.restart_mission()
        elif self.state=="START_MISSION":
            self.leds_publisher.publish("FRONT_KITT_BLUE")
            self.leds_publisher.publish("SIDE_BLUE_WAVE")
        elif self.state=="FOUNDED_FINISH":
            self.leds_publisher.publish("FRONT_BLUE_100")
            self.leds_publisher.publish("SIDE_BLUE_100")



    def start_mode(self):
        rospy.loginfo("Starting autonomy mode...")
        self.state_publisher.publish("START_AUTONOMY_MODE")
        self.prepare_mission()
        self.running = True
        self.state_publisher.publish("READY_AUTONOMY_MODE")
        rospy.loginfo("Prepared mission")

    def stop_mode(self):
        self.destroy_mission()
        self.state_publisher.publish("STOP_AUTONOMY_MODE")



    def prepare_mission(self):
        self.leds_publisher.publish("FRONT_BLUE_BREATH")
        self.leds_publisher.publish("SIDE_BLUE_LOAD")
        self.navigation_process = subprocess.Popen(['roslaunch', 'cyber_is_navigation', 'start_navigation.launch'])
        self.filters_process = subprocess.Popen(['roslaunch', 'cyber_is_filters', 'start_filters.launch'])
        self.mission_process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'mission.launch'])
        rospy.sleep(10)
        self.leds_publisher.publish("SIDE_BLUE_100")

    def destroy_mission(self):
        self.running = False
        if self.navigation_process and self.navigation_process.poll() is None:
            self.navigation_process.terminate()
            self.navigation_process.wait()
            rospy.loginfo("Killing navigation process")
        else:
            rospy.logerr("Not killing navigation process")

        if self.filters_process and self.filters_process.poll() is None:
            self.filters_process.terminate()
            self.filters_process.wait()
            rospy.loginfo("Killing filters process")
        else:
            rospy.logerr("Not killing filters process")

        if self.mission_process and self.mission_process.poll() is None:
            self.mission_process.terminate()
            self.mission_process.wait()
            rospy.loginfo("Killing mission process")
        else:
            rospy.logerr("Not killing mission process")

    def abort_mission(self):
        rospy.loginfo("Aborting mission...")
        self.leds_publisher.publish("SIDE_RED_HAZARD")
        self.leds_publisher.publish("FRONT_RED_SOS")
        self.state_publisher.publish("ABORTED_MISSION")
        self.destroy_mission()
        self.cmd_publisher.publish(Twist())
        self.state_publisher.publish("STOPPED_ROBOT")
        self.state_publisher.publish("WAITING_FOR_RESTART")
        self.leds_publisher.publish("SIDE_RED_100")
        rospy.loginfo("Aborted mission")

    def restart_mission(self):
        rospy.loginfo("Restarting autonomy mode...")
        self.state_publisher.publish("RESTART_AUTONOMY_MODE")
        self.prepare_mission()
        self.running = True
        self.state_publisher.publish("READY_AUTONOMY_MODE")
        rospy.loginfo("Prepared mission")


