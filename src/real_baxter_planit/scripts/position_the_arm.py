#!/usr/bin/env python

import sys
import time
import rospy
import copy
from math import pi, tau
from planit.msg import PercievedObject
from real_baxter_planit import BaxterPlanner
import moveit_commander
from moveit_commander.conversions import *

def position_the_arm():
    offset_x = 0
    offset_y = 0.56
    print('init pos')
    rospy.init_node("baxter_planit", anonymous=False)

    planner = BaxterPlanner(False)

    chirality = 'right'


    # the arm can reach
    # x goes from 0.6 to 1.2
    # y goes from 0 to -0.6

    planner = BaxterPlanner(False)
    # perception_sub = rospy.Subscriber('/perception', PercievedObject, planner.scene.updatePerception)
    time.sleep(1)
    # planner.scene.add_box('table0', list_to_pose_stamped([1.07, 0.0, -0.22, 0, 0, 0], 'world'), (1.0, 1.8, 0.5))
    # planner.scene.add_box('table1', list_to_pose_stamped([0.0, 0.65, -0.43, 0, 0, 0], 'world'), (1.15, 0.5, 0.5))
    # planner.scene.add_box('block', list_to_pose_stamped([1.0, -0.5, -0.05, 0, 0, 0], 'world'), (0.05, 0.05, 0.26))
    planner.scene.add_box('table_base', list_to_pose_stamped([0.9525 + offset_x, -0.23 + offset_y, 0.3825, 0, 0, 0], 'world'), (0.81, 1.2, 0.765))
    planner.scene.add_box('table', list_to_pose_stamped([0.98 + offset_x, -0.23 + offset_y, 0.8675, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    planner.scene.add_box('boundaryW', list_to_pose_stamped([1.225 + offset_x, -0.265 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.09, 0.58, 0.175))
    planner.scene.add_box('boundaryS', list_to_pose_stamped([0.975 + offset_x, 0.07 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.59, 0.09, 0.175))
    planner.scene.add_box('boundaryN', list_to_pose_stamped([0.975 + offset_x, -0.60 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.59, 0.09, 0.175))
    time.sleep(1)

    chirality = 'right'
    

    def straight_movement(direction=[1, 0, 0], length=0.4):
        planner.do_end_effector('close', group_name=chirality + '_hand')
        # print("\033[34m straight move: direction length \033[0m", direction, length)
        plan, planning_time = planner.plan_line_traj(direction, length, group_name=chirality + "_arm")

        planner.execute(plan, group_name=chirality + "_arm")


    # the arm can reach
    # x goes from 0.6 to 1.2
    # y goes from 0 to -0.6

    ################# go to tip position
    # tip_x = 0.75
    # tip_y = -0.6

    # tip_x = 1.0
    # tip_y = -0.6

    # tip_x = 1.0
    # tip_y = 0.06

    # tip_x = 0.75
    # tip_y = 0.06

    tip_x = 0.72
    tip_y = -0.3

    # success, plan, planning_time, error_code = planner.plan_ee_pose(
    #     list_to_pose([tip_x, tip_y+offset_y, 1.24, -pi / 2, 0, -pi / 2]), group_name=chirality + '_arm'
    # )
    success, plan, planning_time, error_code = planner.plan_ee_pose(
        list_to_pose([tip_x, tip_y+offset_y, 1.06, -pi / 2, 0, -pi / 2]), group_name=chirality + '_arm'
    )
    if  success: 
        planner.execute(plan, group_name=chirality + "_arm")


if __name__ == '__main__':
    position_the_arm()