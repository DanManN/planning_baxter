from hashlib import shake_128
import os
import time
import sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(
    os.path.abspath(__file__)), 'src', 'baxter_planit', 'scripts'))
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point


"""Persistent_Homology actions"""
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Persistent_Homology'))

print(os.path.abspath(__file__))
# from src.baxter_planit.scripts.PHIA_1st import PHIA_pipeline as sim_get_plan

from Persistent_Homology.spawn_config import main as spawn_config

# from src.baxter_planit.scripts.stick import spawning
# from src.baxter_planit.scripts.spawn_objects import main as spawning_object


import time
from math import pi, tau, dist


import std_msgs
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import *

import sys
import time
import rospy
from math import pi
# from planit.msg import PercievedObject
from src.baxter_planit.src.baxter_planit.baxter_planner import BaxterPlanner
import numpy as np
from moveit_commander.conversions import *
# from setup_moveit_obstacles import setup_moveit_obstacles

rospy.init_node("baxter_planit", anonymous=False)

planner = BaxterPlanner(False)



""" util """
def write_time_data(scene="data", type_of_plan="unknown", actions=0,
                    time_perception=0,
                    time_get_arm_current_position=0,
                    time_to_plan=0,
                    time_motion_planning=0):
    folder = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "Persistent_Homology", "examples", "")
    name = folder + "data.txt"
    f = open(name, "a")
    f.write(f"{scene} {type_of_plan} {actions} {time_perception} {time_get_arm_current_position} {time_to_plan} {time_motion_planning}\n")
    f.close()

def read_plan():
    plan_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'plan.txt'
    )
    with open(plan_file, 'r') as f:
        line = f.readline().split()
    return line[1], line[-1]

def number_actions():
    plan_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'plan.txt'
    )
    count = 0
    with open(plan_file, 'r') as f:
        for i in f.readlines():
            if i[0:6] == "action":
                count += 1

    return count

def get_workspace_center():
    rospy.wait_for_service('/gazebo/get_model_state')

    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    pose = model_coordinates('boundaryS', 'world')
    x, z = pose.pose.position.x, pose.pose.position.z
    pose = model_coordinates('boundaryW', 'world')
    y = pose.pose.position.y
    print("x,y,z", x, y, z)
    return x, y, z

'''move'''
def straight_movement(direction=[1, 0, 0], length=0.1):
    # planner.do_end_effector('close')
    # print("\033[34m straight move: direction length \033[0m", direction, length)
    plan, planning_time = planner.plan_line_traj(direction, length, group_name="right_arm")
    planner.execute(plan, group_name="right_arm")

def move_action():

    offset_x = 0
    offset_y = 0.56

    # print('read_plan')

    # rospy.init_node("baxter_planit", anonymous=False)

    # planner = BaxterPlanner(False)


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
                    straight_movement(direction=direction, length=length)

    print("execution time:", time.time()-start)

def combine_actions(d1, l1, redundant_vector):
    ''' combine two straight line actions '''
    new_direction = np.array(d1)*l1 + np.array(redundant_vector)
    new_length = np.linalg.norm(new_direction)
    new_direction /= new_length
    return list(new_direction), new_length


def pipeline(type_of_plan, time_to_plan="unknown", scene="unknown"):
    actions = 0
    time_perception = 0
    time_get_arm_current_position = 0
    time_task_planning = 0
    time_motion_planning = 0
    start = time.time()

    total_number_actions = number_actions()

    # real_position_the_arm()

    print('real_withdraw time', time.time()-start)

    while 1:
        start = time.time()
        # real_get_arm_position()
        time_get_arm_current_position += time.time()-start
        start = time.time()
        # type_of_plan()
        time_task_planning += time.time()-start
        print('check arm: ', time_get_arm_current_position)
        print('task planning ', time_task_planning)
        # input('Plan ready')
        start = time.time()
        # print(f" actions = {actions} total = {total_number_actions}")
        if actions >= total_number_actions:
            print('check arm: ', time_get_arm_current_position)
            print('task planning ', time_task_planning)
            print('END')
            break
        print('simulation plan and execute time', time.time()-start)
        move_action()
        actions += total_number_actions
        time_motion_planning += time.time()-start
        print('finish executing')
        print('check arm: ', time_get_arm_current_position)
        print('task planning ', time_task_planning)
        print('motion planning and execution: ', time_motion_planning)

    write_time_data(scene, type_of_plan, actions, time_perception, time_get_arm_current_position, time_to_plan, time_motion_planning)


if __name__ == '__main__':

    scene = "s2" 
    

    # print(sys.argv)
    
    if len(sys.argv)>1:
        if int(sys.argv[1]):  # position the arm
            
            success, plan, planning_time, error_code = planner.plan_ee_pose(
                        [0.72, -0.3, 1.05, -pi / 2, 0, -pi / 2], group_name="right_arm")
            # print(success, planning_time, error_code)
            planner.execute(plan, v_scale=1, group_name="right_arm")

    if len(sys.argv)>2:
        if int(sys.argv[2]):  # run plan
            print("spwan objects")
            spawn_config()

    if len(sys.argv)>3:
        if int(sys.argv[3]):  # run plan
            print("RUN PLAN")
            type_of_plan, time_to_plan = read_plan()
            pipeline(type_of_plan, time_to_plan, scene)

    # success, plan, planning_time, error_code = planner.plan_ee_pose(
    #             [0.72, -0.3, 1.05, -pi / 2, 0, -pi / 2], group_name="right_arm")
    # # print(success, planning_time, error_code)

    # planner.execute(plan, v_scale=1, group_name="right_arm")

