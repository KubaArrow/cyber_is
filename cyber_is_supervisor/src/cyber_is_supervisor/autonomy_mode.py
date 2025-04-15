import rospy
import subprocess
from std_msgs.msg import String

class AutonomyMode:

    def __init__(self):
        self.state = ""
        self.process=None
        self.robot_state_sub=rospy.Subscriber("/robot_state", String, self.callback)
        self.publisher=rospy.Publisher("/robot_state", String, queue_size=10)
        self.running = False
        self.mapping_process=None
        self.move_base_process=None
        self.filters_process=None



    def start_mode(self):
        rospy.loginfo("Starting autonomy mode...")
        self.running = True

    def stop_mode(self):
        rospy.loginfo("Stoping autonomy mode...")
        self.running = False


    def callback(self, data):
        self.state = data.data
        rospy.loginfo(self.state)


    def state_switch(self):
        if self.state == "START_MISSION":
            self.start_mission()

    def prepare_mission(self):
        self.mapping_process = None
        self.move_base_process = None
        self.filters_process = None
        rospy.loginfo("Starting mission...")