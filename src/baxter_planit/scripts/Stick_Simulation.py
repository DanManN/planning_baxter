# Stick_Simulation.py  2022-30-04
# MIT LICENSE 2022 Ewerton R. Vieira

import sys
from math import pi


import rospy
import tf
import threading
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties, GetLinkState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point, Pose
# from geometry_msgs.msg import *
import time
import os
import random
import rospkg
from Connected_Comp import *
import PH_planning

import numpy as np


class Stick_Simulation:

    def __init__(self, ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                 BOUNDARY_S, TABLE, nu, h):

        # self.PH = PH_planning.PH_planning(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
        #              BOUNDARY_S, TABLE, nu, h)

        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/get_link_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.world_properties = rospy.ServiceProxy(
            '/gazebo/get_world_properties', GetWorldProperties)
        # orient = Quaternion(0, 0, 1, 1)

        self.ARM_LENGTH = ARM_LENGTH
        self.RADIUS_OBS = RADIUS_OBS
        self.RADIUS_CC = 0.08   # 0.15  # 0.315
        self.WIDTH_ARM = WIDTH_ARM  # 0.12  # diameter of the cylinder for the wrist
        self.BOUNDARY_N = BOUNDARY_N
        self.BOUNDARY_S = BOUNDARY_S
        self.nu = nu
        self.h = h
        self.TABLE = TABLE  # x distance of the table to the (0,0)

        self.y_shift = 0.56

        # self.phi = self.PH.angle_phi()

    def tip_position(self, model="stick", phi=0):
        """Return the position of the tip of the gripper taking
        in account the orientation"""
        # sign = -np.sign(link_state(link, model).link_state.pose.orientation.x) # not working I need to understand orientation
        sign = 1
        lengh_gripper2elbow = 0.3575  # length of the left finger
        # print("orientation", sign)
        # x, y = self.PH.model_pos("stick")[0:2]
        model_state_stick = self.model_state('stick', "world")
        x, y = model_state_stick.pose.position.x, model_state_stick.pose.position.y

        return [x + np.cos(phi) * lengh_gripper2elbow, y +
                np.sin(phi) * lengh_gripper2elbow]

    def world(self):
        world_positions = dict()

        for i in self.world_properties().model_names:
            # if i[0:5] != "small":
            #     continue
            position = self.model_state(i, "world").pose.position
            world_positions[i] = [position.x, position.y + self.y_shift]
        tip_position = self.tip_position()
        world_positions["tip_gripper_0"] = [tip_position[0], tip_position[1] + self.y_shift]
        return world_positions

    def set_config(self, config):
        for i in self.world_properties().model_names:
            if not any([i[0:6] == "object", i[0:8] == "obstacle", i[0:5] == "stick"]):
                continue

            self.set_model_state(ModelState(i, config[i],
                                            Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0)),
                                            "world"))
            # x, y, z = config[i]
            # self.set_model_state(ModelState(i, Pose(Point(x=x, y=y, z=z), orient),
            #                            Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0)),
            #                            "world"))

    def read_config(self):
        config = dict()
        for i in self.world_properties().model_names:
            # position = self.model_state(i, "world").pose.position
            # config[i] = [position.x, position.y, position.z]
            config[i] = self.model_state(i, "world").pose
        return config

    def straight_movement_stick(self, goal_pose, phi=0, speed=0.1):
        error = 0.005
        success = True

        goal_pose[1] = goal_pose[1] - self.y_shift

        current = np.array(self.tip_position(model="stick", phi=0))
        current_pose = current
        # print("stick_straight_movement:get current pose")
        start = time.time()
        # print("error", np.linalg.norm(current_pose[0:2] - goal_pose[0:2], 2))
        while (np.linalg.norm(current_pose - goal_pose, 2) >= error) and (time.time()-start < 10.0):
            # print(current_pose)
            # print("error", np.linalg.norm(current_pose[0:2] - goal_pose[0:2], 2))
            vel = goal_pose - current
            scalar = speed/np.linalg.norm(vel, 2)
            vel = scalar * vel

            vx, vy = vel

            temp_inf = self.model_state('stick', "world")
            # print(temp_inf)

            # # move setting the position
            # self.set_model_state(ModelState('stick', temp_inf.pose,
            #                            Twist(Vector3(vx, vy, 0), Vector3(0, 0, 0)),
            #                            "world"))

            # move by velocity
            self.set_model_state(ModelState('stick', temp_inf.pose,
                                            Twist(Vector3(vx, vy, 0), Vector3(0, 0, 0)),
                                            "world"))

            # # stop
            # temp_inf = self.model_state('stick', "world")
            # self.set_model_state(ModelState('stick', temp_inf.pose,
            #                            Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)),
            #                            "world"))

            # find position of the tip
            current_pose = np.array(self.tip_position(model="stick", phi=0))
            # current_pose = np.array(
            #     [temp_inf.pose.position.x, temp_inf.pose.position.y])
        # print("stick_straight_movement:end movement, stop")

        # to correct the position
        current = self.model_state('stick', "world")
        # correct the orientationprin
        # current.pose.orientation.x = 1.8763713373577362e-06
        # current.pose.orientation.y = 0.7071090180403072
        # current.pose.orientation.z = 1.8763713373577362e-06
        # current.pose.orientation.w = -0.7071045443207326
        self.set_model_state(ModelState('stick', current.pose,
                                        Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)),
                                        "world"))

        # success &= self.wait_static()

        # print("stick_straight_movement:set position")
        # x, y, z = goal_pose[0], goal_pose[1], goal_pose[2]
        # set_model_state(ModelState('stick', Pose(Point(x, y, z), orient),
        #                            Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)),
        #                            "world"))

        # success &= self.wait_static()
        return success

    def move_rel_tip(self, tip_position, point, phi=0):
        """Move relative to tip 2d, by default tip_position is not needed"""
        # tip = np.array(tip_position(phi=phi))
        # point = np.array([point[0], point[1]])
        # # print("\033[34m move_rel_tip: initial tip position \033[0m", tip)
        #
        # length = np.linalg.norm(point - tip)
        # direction = (point - tip) / length

        print("\033[34m move to point \033[0m", point)
        # if length > 0.03:  # fail to move tiny lengh
        self.straight_movement_stick(point)
        # print("\033[34m move_rel_tip: final tip position \033[0m", np.array(tip_position(phi = phi)))
