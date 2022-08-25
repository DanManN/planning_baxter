import sys
import copy
from math import pi, tau, dist, fabs, cos

import rospy
# import moveit_msgs.msg
# import geometry_msgs.msg
from baxter_core_msgs.msg import EndEffectorCommand

import moveit_commander
from moveit_commander.conversions import *
from planit import Planner


class BaxterPlanner(Planner):

    def __init__(
        self,
        gripper_width=0.042,
        is_sim=False,
        commander_args=['joint_states:=/robot/joint_states']
    ):
        super().__init__(is_sim, commander_args)

        if not is_sim:
            self.eef_pub_left = rospy.Publisher(
                '/robot/end_effector/left_gripper/command',
                EndEffectorCommand,
                queue_size=5,
            )
            self.eef_pub_right = rospy.Publisher(
                '/robot/end_effector/right_gripper/command',
                EndEffectorCommand,
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
            eef_cmd = EndEffectorCommand()
            eef_cmd.id = 65664
            if command == 'open':
                eef_cmd.command = 'go'
                eef_cmd.args = '{\"position\":100.0, \"force\":100}'
            elif command == 'close':
                eef_cmd.command = 'go'
                eef_cmd.args = '{\"position\":0.0, \"force\":100}'
            if group_name == 'left_hand':
                self.eef_pub_left.publish(eef_cmd)
            elif group_name == 'right_hand':
                self.eef_pub_right.publish(eef_cmd)
            else:
                print("Invalid group name!", file=sys.stderr)
