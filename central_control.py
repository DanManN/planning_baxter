from src.real_baxter_planit.scripts.grasp_simple import grasp_simple as grasp_simple
from baxter_interface import CHECK_VERSION
import baxter_interface
from moveit_commander.conversions import *
from geometry_msgs.msg import PoseStamped
import moveit_commander
import std_msgs
import pybullet as p
from math import pi, tau, dist
import copy
from baxter_planit import BaxterPlanner
from planit.msg import PercievedObject
from planit.utils import *
from Persistent_Homology.PHIA import main as PHIA
from Persistent_Homology.PHIM import main as PHIM
from src.real_baxter_planit.scripts.demo_read_plan import straight_movement as straight_movement
from src.real_baxter_planit.scripts.demo_read_plan import demo_real_plan as real_execute
from src.real_baxter_planit.scripts.get_arm_position import get_arm_position as real_get_arm_position
from src.real_baxter_planit.scripts.position_the_arm import position_the_arm as real_position_the_arm
from Perception.pose_estimation_v1_for_letters.get_tags import Perception
from geometry_msgs.msg import Quaternion, Pose, Point
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
import rospy
import os
import time
import numpy as np
import sys
sys.path.insert(0, os.path.join(os.path.dirname(
    os.path.abspath(__file__)), 'src', 'baxter_planit', 'scripts'))

"""perception and real moviments"""
# from src.baxter_planit.scripts.position_the_arm import position_the_arm as sim_position_the_arm

"""Persistent_Homology actions"""
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Persistent_Homology'))

print(os.path.abspath(__file__))
# from src.baxter_planit.scripts.PHIA_1st import PHIA_pipeline as sim_get_plan

# from src.baxter_planit.scripts.stick import spawning
# from src.baxter_planit.scripts.spawn_objects import main as spawning_object

""" Graps functions """

# from grasp import grasp_cylinder

""" functions that require all imported modules"""


def write_time_data(scene="s1", actions=0,
                    time_perception=0,
                    time_get_arm_current_position=0,
                    time_to_plan=0,
                    time_success=0,
                    time_motion_planning=0):
    folder = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "Persistent_Homology", "examples", "")
    name = folder + scene + ".txt"
    f = open(name, "a")
    f.write(f"{scene} {actions} {time_perception} {time_get_arm_current_position} {time_to_plan} {time_success} {time_motion_planning}")
    f.close()


def real_perception(P: Perception):
    # cmd = 'python ./Perception/pose_estimation_v1_for_letters/get_tags.py'
    P.update_locations()


def real_calibration():
    return Perception()


def read_plan():
    plan_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'plan.txt'
    )
    with open(plan_file, 'r') as f:
        line = f.readline().split()
    return line[1], line[-1]


def check_success(actions):
    plan_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'plan.txt'
    )
    with open(plan_file, 'r') as f:
        # each actions counts as 4 lines
        # if len(list(f.readlines())) <= 4*actions:
        if len(list(f.readlines())) > 3:
            return True
        else:
            return False

############################# Utils ##############################


def get_workspace_center():
    rospy.wait_for_service('/gazebo/get_model_state')

    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    pose = model_coordinates('boundaryS', 'world')
    x, z = pose.pose.position.x, pose.pose.position.z
    pose = model_coordinates('boundaryW', 'world')
    y = pose.pose.position.y
    print("x,y,z", x, y, z)
    return x, y, z


def pipeline(type_of_plan, time_to_plan="unknown"):
    actions = 0
    time_perception = 0
    time_get_arm_current_position = 0
    time_task_planning = 0
    time_success = 0
    time_motion_planning = 0
    start = time.time()
    real_position_the_arm()
    print('real_withdraw time', time.time()-start)
    res = input('Enter')
    if res != "":
        print(res)
        print('stopped')
        return
    need_cali = True
    while 1:
        start = time.time()
        if need_cali:
            P = real_calibration()
        real_perception(P)
        need_cali = False
        print('perception')
        print('perception time', time.time()-start)
        time_perception += time.time()-start
        # start = time.time()
        # sim_spawn_objects()
        # print('finish spawning')
        # print('simulation withdraw time' , time.time()-start)
        start = time.time()
        real_get_arm_position()
        time_get_arm_current_position += time.time()-start
        start = time.time()
        # type_of_plan()
        time_task_planning += time.time()-start
        print('perception: ', time_perception)
        print('check arm: ', time_get_arm_current_position)
        print('task planning ', time_task_planning)
        # input('Plan ready')
        start = time.time()
        if check_success(actions):
            print('perception: ', time_perception)
            print('check arm: ', time_get_arm_current_position)
            print('task planning ', time_task_planning)
            print('END')
            break
        print('simulation plan and execute time', time.time()-start)
        real_execute()
        actions += 1
        time_motion_planning += time.time()-start
        print('finish executing')
        print('perception: ', time_perception)
        print('check arm: ', time_get_arm_current_position)
        print('task planning ', time_task_planning)
        print('motion planning and execution: ', time_motion_planning)

        res = input('Enter')
        if res != "":
            print(res)
            print('stopped')
            return

    write_time_data(scene="s1", type_of_plan, actions,
                    time_perception,
                    time_get_arm_current_position,
                    time_to_plan,
                    time_success,
                    time_motion_planning)


if __name__ == '__main__':

    P = real_calibration()
    real_perception(P)
    real_get_arm_position()

    right = baxter_interface.Gripper('right', CHECK_VERSION)
    right.calibrate()
    right.close()

    if sys.argv[0]:  # position the arm
        real_position_the_arm()

    if sys.argv[1]:  # run plan
        type_of_plan, time_to_plan = read_plan()
        pipeline(type_of_plan, time_to_plan)

    if sys.argv[2]:  # grasp
        print(P.update_locations())
        target = P.update_locations()[106]
        #target = [target[0], target[1] + 0.56 - 0.27374944 + 0.296, 1.1]
        target = [target[0], target[1] + 0.56, 1.1]
        grasp_simple(real_get_arm_position())
        # print(target)
