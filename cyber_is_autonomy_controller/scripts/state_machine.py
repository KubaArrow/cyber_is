#!/usr/bin/env python3
import rospy
import smach
import smach_ros
from std_msgs.msg import String
import subprocess

# Globalna zmienna do przechowywania ostatniego stanu robota
robot_state = ""

# Callback do subskrypcji /robot_state
def state_callback(msg):
    global robot_state
    robot_state = msg.data

# Stan: nasłuchiwanie na "START AUTO"
class WaitForStartAuto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_auto'])
        self.sub = rospy.Subscriber('/robot_state', String, state_callback)

    def execute(self, userdata):
        rospy.loginfo("Czekam na START AUTO...")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if robot_state == "START_AUTO":
                rospy.loginfo("Odebrano START AUTO")
                return 'start_auto'
            rate.sleep()

# Stan: uruchamianie launch file
class LaunchSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['launched'])
        self.process = None

    def execute(self, userdata):
        rospy.loginfo("Uruchamiam search_start.launch...")
        self.process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'search_start.launch'])
        rospy.sleep(2)  # dajemy chwilę launchowi
        return 'launched'

# Stan: oczekiwanie na "START_SEARCHED"
class WaitForSearchDone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.sub = rospy.Subscriber('/robot_state', String, state_callback)

    def execute(self, userdata):
        rospy.loginfo("Czekam na START_SEARCHED...")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if robot_state == "START_SEARCHED":

                return 'done'
            rate.sleep()


class StartMapAndMission(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_started'])
        self.proc_map = None
        self.proc_mission = None

    def execute(self, userdata):
        rospy.loginfo("Uruchamiam mission_collecting.launch i search_map_limit.launch...")

        self.proc_map = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'search_map_limit.launch'])
        self.proc_mission = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'mission_collecting.launch'])

        rospy.sleep(2)  # dajmy czas na wystartowanie
        return 'map_started'


class WaitForMapReady(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_ready'])
        self.sub = rospy.Subscriber('/robot_state', String, state_callback)

    def execute(self, userdata):
        rospy.loginfo("Czekam na MAP READY...")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if robot_state == "MAP READY":
                rospy.loginfo("Odebrano MAP READY")
                return 'map_ready'
            rate.sleep()


class WaitForMapReady(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_ready'])
        self.sub = rospy.Subscriber('/robot_state', String, state_callback)

    def execute(self, userdata):
        rospy.loginfo("Czekam na MAP READY...")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if robot_state == "MAP READY":
                rospy.loginfo("Odebrano MAP READY")
                return 'map_ready'
            rate.sleep()


class GoToZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rospy.loginfo("➡️ Przechodzę do strefy docelowej...")
        # Można tu dodać wywołanie actionlib lub serwisu
        return 'reached'


def main():
    rospy.init_node('robot_state_machine')

    sm = smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('WAIT_FOR_AUTO', WaitForStartAuto(),
                               transitions={'start_auto': 'LAUNCH_SEARCH'})

        smach.StateMachine.add('LAUNCH_SEARCH', LaunchSearch(),
                               transitions={'launched': 'WAIT_FOR_SEARCH_DONE'})

        smach.StateMachine.add('WAIT_FOR_SEARCH_DONE', WaitForSearchDone(),
                               transitions={'done': 'START_MAP_AND_MISSION'})

        smach.StateMachine.add('START_MAP_AND_MISSION', StartMapAndMission(),
                               transitions={'map_started': 'WAIT_FOR_MAP_READY'})

        smach.StateMachine.add('WAIT_FOR_MAP_READY', WaitForMapReady(),
                               transitions={'map_ready': 'GO_TO_ZONE'})

        smach.StateMachine.add('GO_TO_ZONE', GoToZone(),
                               transitions={'reached': 'finished'})

    sis = smach_ros.IntrospectionServer('state_machine_viewer', sm, '/SM_ROOT')
    sis.start()

    sm.execute()
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
