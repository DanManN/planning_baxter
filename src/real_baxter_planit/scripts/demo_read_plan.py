#!/usr/bin/env python

from position_the_arm import position_the_arm
import sys
import time
import rospy
import numpy as np
from math import pi, tau
from planit.msg import PercievedObject
from real_baxter_planit import BaxterPlanner
from moveit_commander.conversions import *
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def straight_movement(direction=[1, 0, 0], length=0.4, pause=False):

    planner = BaxterPlanner(False)
    chirality = "right"
    start = time.time()
    planner.do_end_effector('close', group_name=chirality + '_hand')
    # print("\033[34m straight move: direction length \033[0m", direction, length)
    plan, planning_time = planner.plan_line_traj(
        direction, length, group_name=chirality + "_arm", jump_threshold=5.0)
    print("planning time", time.time()-start)

    if pause:
        res = input('Enter')
        if res != "":
            print(res)
            print('stopped')
            return
    planner.execute(plan, group_name=chirality + "_arm", v_scale=0.1)
    print("all", time.time()-start)


def demo_real_plan(pause=False):

    offset_x = 0
    offset_y = 0.56

    print('read_plan')

    rospy.init_node("baxter_planit", anonymous=False)

    planner = BaxterPlanner(False)
    # perception_sub = rospy.Subscriber('/perception', PercievedObject, planner.scene.updatePerception)
    # time.sleep(1)
    # planner.scene.add_box('table0', list_to_pose_stamped([1.07, 0.0, -0.22, 0, 0, 0], 'world'), (1.0, 1.8, 0.5))
    # planner.scene.add_box('table1', list_to_pose_stamped([0.0, 0.65, -0.43, 0, 0, 0], 'world'), (1.15, 0.5, 0.5))
    # planner.scene.add_box('block', list_to_pose_stamped([1.0, -0.5, -0.05, 0, 0, 0], 'world'), (0.05, 0.05, 0.26))
    start = time.time()
    planner.scene.add_box('table_base', list_to_pose_stamped(
        [0.9525 + offset_x, -0.23 + offset_y, 0.3825, 0, 0, 0], 'world'), (0.81, 1.2, 0.765))
    planner.scene.add_box('table', list_to_pose_stamped(
        [0.98 + offset_x, -0.23 + offset_y, 0.8275, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    planner.scene.add_box('boundaryW', list_to_pose_stamped(
        [1.225 + offset_x, -0.265 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.09, 0.58, 0.175))
    planner.scene.add_box('boundaryS', list_to_pose_stamped(
        [0.975 + offset_x, 0.07 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.59, 0.09, 0.175))
    planner.scene.add_box('boundaryN', list_to_pose_stamped(
        [0.975 + offset_x, -0.60 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.59, 0.09, 0.175))
    # time.sleep(1)
    print("loading scene", time.time()-start)

    chirality = 'right'

    # def straight_movement(direction=[1, 0, 0], length=0.4):
    #     start = time.time()
    #     planner.do_end_effector('close', group_name=chirality + '_hand')
    #     # print("\033[34m straight move: direction length \033[0m", direction, length)
    #     plan, planning_time = planner.plan_line_traj(direction, length, group_name=chirality + "_arm",jump_threshold=5.0)
    #     print("planning time", time.time()-start)

    #     res = input('Enter')
    #     if res != "":
    #         print(res)
    #         print('stopped')
    #         return
    #     planner.execute(plan, group_name=chirality + "_arm", v_scale=0.1)
    #     print("all", time.time()-start)

    # the arm can reach
    # x goes from 0.6 to 1.2
    # y goes from 0 to -0.6

    # success, plan, planning_time, error_code = planner.plan_ee_pose(
    #     list_to_pose([0.72, -0.3+offset_y, 1.05, -pi / 2, 0, -pi / 2]), group_name=chirality + '_arm'
    # )
    # if  success:
    #     planner.execute(plan, group_name=chirality + "_arm")

    # position_the_arm()

    # only execute the first action
    second_action = False

    # control robot speed
    execution_steps = 1

    start = time.time()
    # restore actions to reverse the path
    plan_path = '/home/pracsys/retrieval/Kai/Aggregation/plan.txt'
    redundant_vector = np.array([0, 0, 0])
    with open(plan_path, 'r') as f:
        for line in f.readlines():
            if line[0] == 'a' or line[0] == 'p':
                # if second_action:
                #     break # only execute the first action
                # second_action = True
                continue
            else:
                words = line.split(sep='],')
                direction_str = words[0][1:]
                direction = direction_str.split(sep=',')
                direction = [float(s) for s in direction]
                length = float(words[1])

                # if np.linalg.norm(direction[0] - 1) < 0.1:  # add correction
                #     length -= 0.04
                
                direction, length = combine_actions(direction, length, redundant_vector)
                print('direction', direction)
                print('length', length)
                if length <= 0.03:
                    redundant_vector = np.array(direction)*length
                    # input("stop")
                else:
                    redundant_vector = np.array([0, 0, 0])
                    print('execute')
                    # input("stop")
                    straight_movement(direction=direction, length=length, pause=pause)

    print("execution time:", time.time()-start)
    # plan = planner.withdraw(group_name=chirality + "_arm")
    # planner.execute(plan, group_name=chirality + "_arm")

    # straight_movement(direction=[1, 0, 0], length=0.155)

    # straight_movement(direction=[0, 1, 0], length=0.1+0.08)

    # straight_movement(direction=[1, 0, 0], length=0.1)

    # straight_movement(direction=[0, 1, 0], length=-0.2-0.08)

    # straight_movement(direction=[1, 0, 0], length=0.1)

    # ### pick up block ###
    # planner.pick('block', v_scale=0.25, a_scale=1.0, grasping_group=chirality + "_hand", group_name=chirality + "_arm")
    # # planner.do_end_effector('open', group_name=chirality + '_hand')

    # ### place block ###
    # # place_poses = planner.grasps.get_simple_placements([1.0, 0.0, -0.1])
    # # success, plan, planning_time, error_code = planner.plan_ee_poses(place_poses, group_name=chirality + '_arm')
    # success, plan, planning_time, error_code = planner.plan_ee_pose(
    #     list_to_pose([1.0, 0.0, -0.04, tau / 4, tau / 4, tau / 4]), group_name=chirality + '_arm'
    # )
    # print(success, planning_time, error_code)
    # if not success:
    #     sys.exit()
    # planner.execute(plan, v_scale=0.25, group_name=chirality + '_arm')
    # print("Preplaced")
    # planner.do_end_effector('open', group_name=chirality + '_hand')
    # print("Gripper Opened")
    # success = planner.detach('block', group_name=chirality + "_arm")
    # print("Object detached:", success)
    # plan, fraction = planner.plan_line_traj([0, 0, 1], 0.18, group_name=chirality + '_arm', avoid_collisions=False)
    # print("PostPlace", fraction)
    # if fraction < 0.5:
    #     sys.exit()
    # planner.execute(plan, v_scale=0.125, group_name=chirality + '_arm')
    # print("PostPlaced", fraction)


def combine_actions(d1, l1, redundant_vector):
    ''' combine two straight line actions '''
    new_direction = np.array(d1)*l1 + np.array(redundant_vector)
    new_length = np.linalg.norm(new_direction)
    new_direction /= new_length
    return list(new_direction), new_length
