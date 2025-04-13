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

class MissionAborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['aborted'])

    def execute(self, userdata):
        rospy.logwarn("🛑 Misja przerwana przez STOP_MISSION!")
        return 'aborted'

class AbortableState(smach.State):
    def __init__(self, outcomes):
        super(AbortableState, self).__init__(outcomes=outcomes + ['abort'])

    def check_abort(self):
        if robot_state == "STOP_MISSION":
            rospy.logwarn("Wykryto STOP_MISSION! Przerywam misję.")
            return True
        return False



class WaitForStartAuto(smach.State):
    def __init__(self):
        super().__init__(outcomes=['start_auto'])
        self.sub = rospy.Subscriber('/robot_state', String, state_callback)
        self.pub = rospy.Publisher('/robot_state', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("Czekam na START AUTO...")
        rate = rospy.Rate(5)
        rospy.sleep(0.5)
        self.pub.publish("NOT_STARTED")

        while not rospy.is_shutdown():
            if robot_state == "START_MISSION":
                rospy.loginfo("Odebrano START AUTO")
                return 'start_auto'
            rate.sleep()

class LaunchSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['launched'])
        self.process = None

    def execute(self, userdata):
        rospy.loginfo("Uruchamiam search_start.launch...")
        self.process = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'search_start.launch'])
        rospy.sleep(2)  # dajemy chwilę launchowi
        return 'launched'

class WaitForSearchDone(AbortableState):
    def __init__(self):
        super().__init__(outcomes=['done'])
        self.sub = rospy.Subscriber('/robot_state', String, state_callback)

    def execute(self, userdata):
        rospy.loginfo("Czekam na START_SEARCHED...")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.check_abort():
                return 'abort'
            if robot_state == "START_SEARCHED":
                return 'done'
            rate.sleep()

class WaitForFailedMission(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failed_ack'])

        self.sub = rospy.Subscriber('/robot_state', String, state_callback)

    def execute(self, userdata):
        rospy.loginfo("Czekam na FAILED_MISSION...")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if robot_state == "FAILED_MISSION":
                rospy.logwarn("💥 Otrzymano FAILED_MISSION")
                return 'failed_ack'
            rate.sleep()




class StartMapAndMission(smach.State):  # ✅ zamiast AbortableState
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_started'])
        self.proc_map = None
        self.proc_mission = None

    def execute(self, userdata):
        rospy.loginfo("Uruchamiam mission_collecting.launch i search_map_limit.launch...")

        self.proc_map = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'search_map_limit.launch'])
        self.proc_mission = subprocess.Popen(['roslaunch', 'cyber_is_mission_elements', 'mission_collecting.launch'])

        rospy.sleep(2)
        return 'map_started'



class WaitForMapReady(AbortableState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_ready'])
        self.sub = rospy.Subscriber('/robot_state', String, state_callback)

    def execute(self, userdata):
        rospy.loginfo("Czekam na MAP READY...")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.check_abort():
                return 'abort'
            if robot_state == "MAP_READY":
                return 'done'
            rate.sleep()


class WaitForMapReady(AbortableState):
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




def main():
    rospy.init_node('robot_state_machine')

    sm = smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('WAIT_FOR_FAILED_MISSION', WaitForFailedMission(),
                               transitions={'failed_ack': 'finished'})

        smach.StateMachine.add('MISSION_ABORTED', MissionAborted(),
                               transitions={'aborted': 'WAIT_FOR_FAILED_MISSION'})

        smach.StateMachine.add('WAIT_FOR_START', WaitForStartAuto(),
                               transitions={'start_auto': 'LAUNCH_SEARCH'})

        smach.StateMachine.add('LAUNCH_SEARCH', LaunchSearch(),
                               transitions={'launched': 'WAIT_FOR_SEARCH_DONE',  'abort': 'MISSION_ABORTED'})

        smach.StateMachine.add('WAIT_FOR_SEARCH_DONE', WaitForSearchDone(),
                               transitions={'done': 'START_MAP_AND_MISSION',  'abort': 'MISSION_ABORTED'})

        smach.StateMachine.add('START_MAP_AND_MISSION', StartMapAndMission(),
                               transitions={'map_started': 'WAIT_FOR_MAP_READY',  'abort': 'MISSION_ABORTED'})

        smach.StateMachine.add('WAIT_FOR_MAP_READY', WaitForMapReady(),
                               transitions={ 'abort': 'MISSION_ABORTED'})


    sis = smach_ros.IntrospectionServer('state_machine_viewer', sm, '/SM_ROOT')
    sis.start()

    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
