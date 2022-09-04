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
import baxter_interface
from baxter_interface import CHECK_VERSION

def grasp_simple(position_tip, position_target, pause=False):
    right = baxter_interface.Gripper('right', CHECK_VERSION)
    # right.calibrate()
    offset_x = 0
    offset_y = 0.56

    position_tip = list(position_tip)
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

    planner.scene.add_box('table_base', list_to_pose_stamped([0.9525 + offset_x, -0.23 + offset_y, 0.3825, 0, 0, 0], 'world'), (0.81, 1.2, 0.765))
    planner.scene.add_box('table', list_to_pose_stamped([0.98 + offset_x, -0.23 + offset_y, 0.88, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    # planner.scene.add_box('shelf_top', list_to_pose_stamped([0.98 + offset_x, -0.23 + offset_y, 0.5+0.8275, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    planner.scene.add_box('boundaryW', list_to_pose_stamped([1.225 + offset_x, -0.265 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.09, 0.58, 0.175))
    planner.scene.add_box('boundaryS', list_to_pose_stamped([0.975 + offset_x, 0.07 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.65, 0.1, 0.275))
    planner.scene.add_box('boundaryN', list_to_pose_stamped([0.975 + offset_x, -0.60 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.65, 0.1, 0.275))
    # perception_sub = rospy.Subscriber(
    #     '/perception', PercievedObject, planner.scene.updatePerception
    # )
    
    chirality = 'right'
    

    def straight_movement(direction=[1, 0, 0], length=0.4, pause=False):
        planner.do_end_effector('close', group_name=chirality + '_hand')
        # print("\033[34m straight move: direction length \033[0m", direction, length)
        plan, planning_time = planner.plan_line_traj(direction, length, group_name=chirality + "_arm")

        if pause:
            res = input('Execute? Enter')
            if res != "":
                print(res)
                print('stopped')
                return

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
    
    move_group = moveit_commander.MoveGroupCommander(chirality + '_arm')
    # move_group.set_support_surface_name("table__linksurface")

    wpose = move_group.get_current_pose().pose
    new_pose = copy.deepcopy(wpose)
    new_pose.position.x = tip_x
    new_pose.position.y = tip_y + offset_y
    new_pose.position.z = 1.05
    waypoints = [new_pose]

    # plan, fraction = move_group.compute_cartesian_path(
    #     waypoints, 0.005, 0.0, avoid_collisions=True
    # )

    # planner.execute(plan, group_name=chirality + "_arm")

    orientation = [-pi / 2, -pi/2, -pi / 2] # orientation to grasp
    # orientation = [-pi / 2, 0, -pi / 2] # original orientation
    tip_z = 1.06



    position_tip[1] += offset_y

    

    # aim to target

    y_aim = position_target[1] - position_tip[1]

    y_aim_length = abs(y_aim)

    y_aim_sign = y_aim/y_aim_length
    correction = 0.02#0.012

    y_aim_length -= y_aim_sign * correction

    straight_movement(direction=[0, y_aim_sign, 0], length=y_aim_length, pause=pause)



    # rotate wrist fixing target position.y

    position_tip[1] = position_target[1]

    success, plan, planning_time, error_code = planner.plan_ee_pose(
        list_to_pose(position_tip + orientation), group_name=chirality + '_arm'
    )


    if pause:
        res = input('Execute? Enter')
        if res != "":
            print(res)
            print('stopped')
            return

    planner.execute(plan, group_name=chirality + "_arm")


    # approach
    right = baxter_interface.Gripper('right', CHECK_VERSION)
    #right.calibrate()
    right.open()

    if pause:
        res = input('Execute? Enter')
        if res != "":
            print(res)
            print('stopped')
            return

    correction_x = 0.022

    x_approach_length = position_target[0] - position_tip[0] + correction_x
    

    straight_movement(direction=[1, 0, 0], length=x_approach_length, pause=pause)

    right.close()

    straight_movement(direction=[0, 0, 1], length=0.065, pause=pause)

    straight_movement(direction=[-1, 0, 0], length=0.35, pause=pause)








if __name__ == '__main__':
    grasp_simple()