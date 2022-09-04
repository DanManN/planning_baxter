# Stick_Simulation.py  2022-30-04
# MIT LICENSE 2022 Ewerton R. Vieira

import sys
import os
from math import pi


import rospy
import tf
import threading
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties, GetLinkState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point, Pose
# from geometry_msgs.msg import *
import time
import random
import rospkg
# from Connected_Comp import *
# import PH_planning

import numpy as np

from std_srvs.srv import Empty
pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)


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

        self.read_initial_position()

        # self.phi = self.PH.angle_phi()

    def read_initial_position(self):
        self.initial_position = dict()
        for i in self.world_properties().model_names:
            self.initial_position[i] = self.model_state(i, "world").pose.position

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
        self.read_initial_position()  # update inital positions

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
        # set first stick than all other objects
        self.set_model_state(ModelState("stick", config["stick"],
                                        Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0)),
                                        "world"))
        pause_physics_client()
        for i in self.world_properties().model_names:
            if not any([i[0:6] == "object", i[0:8] == "obstacle", i[:5] == 'sitck', i[0:8] == "boundary"]):
                continue

            self.set_model_state(ModelState(i, config[i],
                                            Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0)),
                                            "world"))

        # self.set_model_state(ModelState("stick", config["stick"],
        #                                 Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0)),
        #                                 "world"))
        unpause_physics_client()

    def read_config(self):
        config = dict()
        pause_physics_client()
        self.read_initial_position()  # update inital positions

        for i in self.world_properties().model_names:
            # position = self.model_state(i, "world").pose.position
            # config[i] = [position.x, position.y, position.z]
            config[i] = self.model_state(i, "world").pose

        unpause_physics_client()
        return config

    def straight_movement_stick(self, goal_pose, phi=0, speed=0.1):
        error = 0.01
        goal_pose[1] = goal_pose[1] - self.y_shift
        tip_pose = np.array(self.tip_position(model="stick", phi=0))
        # print(tip_pose, " ->", goal_pose)
        start = time.time()
        while (np.linalg.norm(tip_pose - goal_pose, 2) >= error) and (time.time()-start < 10.0):
            vel = goal_pose - tip_pose
            vel = vel * speed / np.linalg.norm(vel, 2)
            vx, vy = vel
            current_state = self.model_state('stick', "world")
            self.set_model_state(ModelState('stick', current_state.pose,
                                            Twist(Vector3(vx, vy, 0), Vector3(0, 0, 0)),
                                            "world"))  # move by velocity
            # find position of the tip
            tip_pose = np.array(self.tip_position(model="stick", phi=0))

        # to correct the position
        current_state = self.model_state('stick', "world")
        self.set_model_state(ModelState('stick', current_state.pose,
                                        Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)),
                                        "world"))
        tip_pose = np.array(self.tip_position(model="stick", phi=0))

        # success &= self.wait_static()

        return np.linalg.norm(tip_pose - goal_pose, 2) < error

    def move_rel_tip(self, tip_position, point, phi=0):
        """Move relative to tip 2d, by default tip_position is not needed"""
        # tip = np.array(tip_position(phi=phi))
        # point = np.array([point[0], point[1]])
        # # print("\033[34m move_rel_tip: initial tip position \033[0m", tip)
        #
        # length = np.linalg.norm(point - tip)
        # direction = (point - tip) / length

        # print("\033[34m move to point \033[0m", point)

        # if length > 0.03:  # fail to move tiny lengh
        success = False
        start = time.time()
        while (not success) and (time.time()-start < 50.0):
            success = self.straight_movement_stick(point)
        # print("\033[34m move_rel_tip: final tip position \033[0m", np.array(tip_position(phi = phi)))

        # success *= self.is_feasible()  # already checking this after group of 3 pushes
        return success

    def points2direction(self, pt_inital, pt_end):
        pt_inital = np.array([pt_inital[0], pt_inital[1]])
        pt_end = np.array([pt_end[0], pt_end[1]])
        length = np.linalg.norm(pt_end - pt_inital)
        direction = (pt_end - pt_inital) / length
        return str([direction[0], direction[1], 0])+", " + str(length)

    def write_plan(self, action_list, number_of_acts=3, name_plan="unknown", time_to_plan="unknown"):

        f = open("plan.txt", "w")
        f.write(
            f"plan: {name_plan} time_to_plan: {time_to_plan}\n")
        for action in action_list:
            f.write("actions\n")
            for j in range(number_of_acts):
                f.write(self.points2direction(action[j], action[j+1]) + "\n")
        f.close()

    def execute_plan(self, action_list):
        for action in action_list:
            for j in range(3):
                self.move_rel_tip(self.tip_position(), action[j+1])

    def is_feasible(self, error_objects=0.1, error_boundary=0.01):
        """return true if no object is being toppled
        and walls are not being displaced"""

        for i in self.world_properties().model_names:
            if i[0:2] == "ob":
                z = self.model_state(i, "world").pose.position.z
                if np.linalg.norm(self.initial_position[i].z - z) > error_objects:
                    print(f"objects {np.linalg.norm(self.initial_position[i].z - z)}")
                    return False
            elif i[0:8] == "boundary":
                p = self.initial_position[i]
                p = np.array([p.x, p.y, p.z])
                q = self.model_state(i, "world").pose.position
                q = np.array([q.x, q.y, q.z])
                if np.linalg.norm(q-p) > error_boundary:
                    print(f"boundary {np.linalg.norm(q-p)}")
                    return False
        return True

    def move_near_to_target(self, error=0.1, target="object_0"):
        """Return true if gripper is near to the target in simulation"""
        position = self.model_state(target, "world").pose.position
        position = [position.x, position.y]
        position_closer = [position[0] - 1.2*self.RADIUS_OBS, position[1] + self.y_shift]

        print(self.tip_position(), position_closer)

        self.move_rel_tip(self.tip_position(), position_closer)

        distance = np.linalg.norm(np.array(self.tip_position()) - np.array(position))

        return distance < error
