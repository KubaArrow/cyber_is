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
        self.mission_process = None
        self.colletor_process = None

        self.shutdown_requested = False

    def callback(self, data):
        if self.running:
            self.state = data.data
            self.state_switch()

    def state_switch(self):
        if self.state == "ABORT_MISSION":
            self.abort_mission()
        elif self.state == "RESTART_MISSION":
            self.restart_mission()
        elif self.state == "START_MISSION":
            self.start_mission()
        elif self.state == "FOUNDED_START_POSE":
            self.start_orientation()
            self.start_mission_collector()
        elif self.state == "FOUNDED_ORIENTATION":
            self.go_to_zone()
        elif self.state == "FOUNDED_ZONE":
            self.search_meta()
        elif self.state == "FOUNDED_FINISH":
            self.finish_mission()
        elif self.state == "END_MISSION":
            self.end_mission()


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
        self.leds_publisher.publish("FRONT_MIN")
        self.leds_publisher.publish("SIDE_BLUE_STROBE")
        self.navigation_process = subprocess.Popen(['roslaunch', 'cyber_is_navigation', 'start_navigation.launch'])
        self.filters_process = subprocess.Popen(['roslaunch', 'cyber_is_filters', 'start_filters.launch'])
        rospy.sleep(10)
        self.leds_publisher.publish("SIDE_BLUE")

    def destroy_mission(self):
        self.running = False
        self.shutdown_requested = True
        if self.navigation_process and self.navigation_process.poll() is None:
            self.navigation_process.terminate()
            self.navigation_process.wait()
        if self.filters_process and self.filters_process.poll() is None:
            self.filters_process.terminate()
            self.filters_process.wait()
        self.kill_mission_process()
        if self.colletor_process and self.colletor_process.poll() is None:
            self.colletor_process.terminate()
            self.colletor_process.wait()


    def abort_mission(self):
        rospy.loginfo("Aborting mission...")
        self.leds_publisher.publish("SIDE_RED")
        self.state_publisher.publish("ABORTED_MISSION")
        self.destroy_mission()
        self.cmd_publisher.publish(Twist())
        self.state_publisher.publish("STOPPED_ROBOT")
        self.state_publisher.publish("WAITING_FOR_RESTART")
        self.leds_publisher.publish("SIDE_RED_STROBE")
        rospy.loginfo("Aborted mission")

    def restart_mission(self):
        rospy.loginfo("Restarting autonomy mode...")
        self.state_publisher.publish("RESTART_AUTONOMY_MODE")
        self.prepare_mission()
        self.running = True
        self.state_publisher.publish("READY_AUTONOMY_MODE")
        rospy.loginfo("Prepared mission")

    def start_mission(self):
        rospy.loginfo("Starting mission")
        self.mission_process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'search_start.launch'])
        rospy.loginfo("Starting search start...")

    def start_orientation(self):
        self.kill_mission_process()
        rospy.loginfo("Starting orientation")
        self.mission_process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'search_orientation.launch'])


    def start_mission_collector(self):
        rospy.loginfo("Starting mission collector")
        self.colletor_process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'mission_collecting.launch'])

    def stop_mission_collector(self):
        if self.colletor_process and self.colletor_process.poll() is None:
            self.colletor_process.terminate()
            self.colletor_process.wait()
            rospy.loginfo("Killing mission process")
        else:
            rospy.loginfo("Not killing mission process")

    def go_to_zone(self):
        self.kill_mission_process()
        rospy.loginfo("Starting go to zone")
        self.mission_process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'go_to_zone.launch'])

    def search_meta(self):
        self.kill_mission_process()
        rospy.loginfo("Starting searching for meta...")
        self.mission_process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'search_meta.launch'])

    def finish_mission(self):
        self.kill_mission_process()
        rospy.loginfo("Starting finish mission")
        self.mission_process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'finish.launch'])
        pass

    def end_mission(self):
        self.destroy_mission()
        rospy.loginfo("Completed mission")
        self.state_publisher.publish("MISSION_COMPLETE")
        self.leds_publisher.publish("SIDE_BLUE_BREATH")
        pass


    def kill_mission_process(self):
        if self.mission_process and self.mission_process.poll() is None:
            self.mission_process.terminate()
            self.mission_process.wait()
            rospy.loginfo("Killing mission process")
        else:
            rospy.loginfo("Not killing mission process")

    # self.monitor_navigation_thread = threading.Thread(target=self.monitor_navigation_process)
    # self.monitor_navigation_thread.daemon = True
    # self.monitor_navigation_thread.start()
    # def monitor_navigation_process(self):
    #     while self.running:
    #         time.sleep(2)
    #         if self.navigation_process and self.navigation_process.poll() is not None:
    #             if self.shutdown_requested:
    #                 rospy.loginfo("Navigation  terminated by user request.")
    #             else:
    #                 rospy.logerr("Navigation  process crashed.")
    #                 self.state_publisher.publish("ERROR_AUTONOMY_MODE")
    #
    #                 # restart only if it wasn't shut down manually
    #                 time.sleep(3)
    #                 rospy.loginfo("Restarting  ...")
    #                 self.start_mode()
    #             self.running = False
    #             break

