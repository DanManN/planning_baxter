#!/usr/bin/env python

import sys
import time
import rospy
from math import pi, tau
from planit.msg import PercievedObject
from baxter_planit import BaxterPlanner
from moveit_commander.conversions import *

from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point
import os
import copy

rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')

get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', SetModelState)


def setup_moveit_obstacles():
    offset_x = 0
    offset_y = 0.0
    
    rospy.init_node("baxter_planit", anonymous=False)

    planner = BaxterPlanner(False)
    # perception_sub = rospy.Subscriber('/perception', PercievedObject, planner.scene.updatePerception)
    time.sleep(1)
    planner.scene.add_box('table_base', list_to_pose_stamped([0.9525 + offset_x, -0.23 + offset_y, 0.3825, 0, 0, 0], 'world'), (0.81, 1.2, 0.765))
    planner.scene.add_box('table', list_to_pose_stamped([0.98 + offset_x, -0.23 + offset_y, 0.8675, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    planner.scene.add_box('boundaryW', list_to_pose_stamped([1.225 + offset_x, -0.265 + offset_y, 1.0275, 0, 0, 0], 'world'), (0.09, 0.58, 0.115))
    planner.scene.add_box('boundaryS', list_to_pose_stamped([0.975 + offset_x, 0.07 + offset_y, 1.0275, 0, 0, 0], 'world'), (0.59, 0.09, 0.115))
    planner.scene.add_box('boundaryN', list_to_pose_stamped([0.975 + offset_x, -0.60 + offset_y, 1.0275, 0, 0, 0], 'world'), (0.59, 0.09, 0.115))
    # planner.scene.add_box('table1', list_to_pose_stamped([0.0, 0.65, -0.43, 0, 0, 0], 'world'), (1.15, 0.5, 0.5))
    # planner.scene.add_box('block', list_to_pose_stamped([1.0, -0.5, -0.05, 0, 0, 0], 'world'), (0.05, 0.05, 0.26))
    time.sleep(1)
