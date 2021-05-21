import sys
import copy
from math import pi, tau, dist, fabs, cos

import rospy
import moveit_msgs.msg
# import geometry_msgs.msg
import baxter_core_msgs.msg

import moveit_commander
from moveit_commander.conversions import *
from planit import Planner


class BaxterPlanner(Planner):
    def __init__(self, is_sim=True, commander_args=['joint_states:=/robot/joint_states']):
        super().__init__(is_sim, commander_args)

        if not is_sim:
            self.eef_pub = rospy.Publisher(
                '/robot/end_effector/left_gripper/command',
                baxter_core_msgs.msg.EndEffectorCommand,
                queue_size=5,
            )

    def do_end_effector(self, command, group_name="left_hand"):
        if self.is_sim:
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joint_goal = move_group.get_current_joint_values()

            if command == 'open':
                joint_goal = [0.02, -0.02]
            elif command == 'close':
                joint_goal = [0.0, 0.0]

            move_group.go(joint_goal, wait=True)
            move_group.stop()
        else:
            eef_cmd = baxter_core_msgs.msg.EndEffectorCommand()
            if command == 'open':
                eef_cmd.command = 'go'
                eef_cmd.args = '{\"position\":100.0}'
            elif command == 'close':
                eef_cmd.command = 'go'
                eef_cmd.args = '{\"position\":0.0}'
            self.eef_pub.publish(eef_cmd)