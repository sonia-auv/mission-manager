#! /usr/bin/env python2

import os
import rospy
import rospkg

from sonia_common.srv import ListMissions, ListMissionsResponse

from sonia_common.msg import MissionNameMsg
from provider_kill_mission.msg import MissionSwitchMsg
from flexbe_msgs.msg import BehaviorExecutionActionGoal
from std_msgs.msg import Empty
from xml.dom import minidom

class MissionManager:

    def __init__(self):
        rospy.init_node('mission_manager')

        self.current_mission = None

        # Get manifest folder
        rp = rospkg.RosPack()
        self.missions_directory = os.path.join(rp.get_path('auv8_mission_controller_flexbe_behaviors'), 'manifest')

        # Subscribers
        self.mission_switch_sub = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg,
                                                    self.handle_mission_switch_activated)
        self.mission_name_sub = rospy.Subscriber('/mission_manager/mission_name_msg', MissionNameMsg, 
                                                    self.handle_current_mission)
        
        # Publishers
        self.flexbe_behavior_pub = rospy.Publisher('/flexbe/execute_behavior/goal', BehaviorExecutionActionGoal, queue_size=5)
        self.flexbe_preempt_pub = rospy.Publisher('/flexbe/command/preempt', Empty, queue_size=100)

        # Services
        rospy.Service('mission_executor/list_missions', ListMissions, self.handle_list_missions)

        self.handle_list_missions(None)

        rospy.spin()

    # Handler for mission switch ON/OFF
    # - msg: Input message from mission switch topic
    # - return: 
    def handle_mission_switch_activated(self, msg):
        rospy.loginfo("Mission switch state change: {}".format(msg.state))
        rospy.loginfo("Mission: {}".format(self.current_mission))
        if self.current_mission:
            if msg.state:
                self.handle_start_mission(None)
            else:
                self.handle_stop_mission(None)
    
    # Handler to get selected mission name
    # - req: Requested mission name.
    # - return: 
    def handle_current_mission(self, req):
        rospy.loginfo("Mission selected : {}".format(req.name))
        self.current_mission = req.name

    # Handler to start the mission
    # - req: Parameter not used.
    # - return:
    def handle_start_mission(self, req):
        rospy.loginfo("Start mission")
        msg = BehaviorExecutionActionGoal()
        msg.goal.behavior_name = self.current_mission
        self.flexbe_behavior_pub.publish(msg)

    # Handler to stop the mission
    # - req: Parameter not used.
    # - return:
    def handle_stop_mission(self, req):
        rospy.loginfo("Stop mission")
        self.flexbe_preempt_pub.publish()

    # Handler to list every available missions
    # - req: Parameter not used.
    # - return: Mission list as a service response.
    def handle_list_missions(self, req):
        rospy.loginfo("List mission")
        missions_list = ""
        for file in os.listdir(self.missions_directory):
            mission_file = minidom.parse(os.path.join(self.missions_directory, file))
            mission = mission_file.getElementsByTagName('behavior')
            mission_name = mission[0].attributes['name'].value
            missions_list = missions_list + mission_name + ";"
        return ListMissionsResponse(missions = missions_list)

if __name__ == '__main__':
    MissionManager()