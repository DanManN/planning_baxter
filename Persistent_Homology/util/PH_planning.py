# PH_planning.py  2022-30-04
# MIT LICENSE 2022 Ewerton R. Vieira

import sys
from math import pi

import tf
# from geometry_msgs.msg import Quaternion

import warnings

# from geometry_msgs.msg import *
import time
import os
import random
from .Connected_Comp import *

import numpy as np
from ripser import ripser, Rips


class PH_planning:

    def __init__(self, ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                 BOUNDARY_S, TABLE, nu, h, world="config.txt"):
        """world can be either a file or a dict()"""

        balls_arr = []

        self.ARM_LENGTH = ARM_LENGTH
        self.RADIUS_OBS = RADIUS_OBS
        self.WIDTH_ARM = WIDTH_ARM  # 0.12  # diameter of the cylinder for the wrist
        self.BOUNDARY_N = BOUNDARY_N
        self.BOUNDARY_S = BOUNDARY_S
        self.nu = nu
        self.h = h
        self.TABLE = TABLE  # x distance of the table to the (0,0)

        self.y_shift = 0.56

        self.paralell = 1.1  # 10% extra to RADIUS_OBS to push objects close to the target. See push_planning function

        self.world = world
        self.write = False

        # self.phi = self.angle_phi()

        if type(world) == type("string"):
            self.write = True
            self.world = self.read_world(world)

        # compute: path region, closest_pt, Connected Components and persistent radii
        self.update()
        # print(f"CC_nu = {self.CC_nu}")

    def update(self):
        # compute Connected Components and persistent radii
        self.path_region, self.closest_pt = self.compute_path_region()

        if self.path_region:  # compute PH for nonempty set
            self.CC_nu, self.radii_ = self.persistent_radii_CC(self.path_region)
            self.radii = self.persistent_radii()
        else:
            self.CC_nu = dict()
            self.radii = None

    def read_world(self, world):

        positions_file = open(world, 'r')
        w = dict()
        obj_index = 0
        obs_index = 0
        for line in positions_file.readlines():
            print(line)
            if (line == "object\n"):
                name = line[0:-1]
                index = 0

            elif (line == "obstacle\n"):
                name = line[0:-1]
                index = 0

            elif (line == "tip_gripper\n"):
                name = line[0:-1]
                index = 0

            else:
                pos = line.split()
                w[name+"_"+str(index)] = [float(pos[0]), float(pos[1]) + self.y_shift]
                index += 1

        positions_file.close()

        return w

    def persistent_radii_CC(self, Obs):
        rips = Rips(verbose=False)
        # print("Obs", Obs)
        Obs = np.array(Obs)
        # print(Obs, "\n", Obs.shape)

        with warnings.catch_warnings():  # remove warinig for one point data cloud
            warnings.simplefilter("ignore")
            diagrams = rips.fit_transform(Obs)

        CC_nu_, radius_diagram = persistent_CC_r(Obs, diagrams[0], self.nu)
        # print(radius_diagram)
        CC_nu = dict()
        for k, radius in enumerate(radius_diagram):  # change from list to dict
            CC_nu[radius] = CC_nu_[k]

        return CC_nu, radius_diagram

    def min_radius_CC(self):
        if self.radii:
            return min(self.radii)
        else:
            return 0

    def persistent_radii(self):
        """return persistent radii > self.h """
        radii = []
        for i in self.radii_:
            if i > self.h:
                radii.append(i)

        return radii if radii else [self.WIDTH_ARM]

    def number_CC(self, radius):
        """Return the number of connected componnets for a given persistent radius"""
        number = len(self.path_region)
        for i in list(self.CC_nu.keys()):
            if i < radius:
                number = len(self.CC_nu[i].keys())
            else:
                break
        return number

    def write_data(self, count_act, planner_time, time_sim, checker, alg):
        with open("stats" + "_" + alg + ".txt", 'a') as file:
            planner_succ = 1
            sim_succ = 0

            if checker == 0:
                sim_succ = 1
            if checker == []:
                sim_succ = 1

            if planner_time > 300:
                planner_succ = 0
                sim_succ = 0

            file.write(str(count_act) + ", " + str(planner_time) + ", " + str(time_sim) +
                       ", " + str(planner_succ) + ", " + str(sim_succ) + "\n")

    def e_from_q(self, quaternion):
        return tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

    def q_from_e(self, r, p, y):
        return tf.transformations.quaternion_from_euler([r, p, y])

    def rot(self, theta):
        """Rotation matrix """
        c_theta, s_theta = np.cos(theta), np.sin(theta)
        return np.array(((c_theta, -s_theta), (s_theta, c_theta)))

    def rot_trans(self, point, phi):
        """From rotated space to state space"""
        R = self.rot(phi)

        if phi < 0:
            T = np.array(point) - np.array([0, 2 * self.WIDTH_ARM])
            P = np.matmul(R, T.reshape(2, 1)) + \
                np.array([self.TABLE, self.BOUNDARY_N]).reshape(2, 1)
        else:
            T = np.array(point)
            P = np.matmul(R, T.reshape(2, 1)) + np.array([self.TABLE, 0]).reshape(2, 1)
        return [P[0, 0], P[1, 0]]

    def trans_rot(self, point, phi):
        """Inverse of rot_trans"""
        R = self.rot(-phi)
        if phi < 0:
            T = np.array(point) - np.array([self.TABLE, self.BOUNDARY_N])
            P = np.matmul(R, T.reshape(2, 1)) + np.array([0, 2 * self.WIDTH_ARM]).reshape(2, 1)
        else:
            T = np.array(point) - np.array([self.TABLE, 0])
            P = np.matmul(R, T.reshape(2, 1))
        return [P[0, 0], P[1, 0]]

    def tip_position(self):
        """Return the position of the tip of the gripper"""
        return self.world["tip_gripper_0"]

    def model_pos(self, i):
        """Return position of the model"""
        return self.world[i]

    def is_close_to_wall(self, target='object_0'):
        if self.WIDTH_ARM <= self.model_pos(target)[1] <= self.BOUNDARY_N - self.WIDTH_ARM:
            return False
        else:
            return True

    def angle_phi(self):
        """Given position of the target, the size of the shelf and the width of the arm
        return the angle_phi"""
        if not self.is_close_to_wall():
            return 0

        # -self.TABLE since the table is self.TABLE far from the axis x
        x = self.model_pos('object_0')[0] - self.TABLE
        y = self.model_pos('object_0')[1]

        # target closer to north wall ( higher value for y)
        if y > self.BOUNDARY_N - self.WIDTH_ARM:
            b = (self.BOUNDARY_N - y)
            sign = 1
        else:
            b = y
            sign = -1  # negative since phi has to be negative in this case
        t = (self.BOUNDARY_N - b)**2

        return sign * np.arcsin((-self.WIDTH_ARM * x + np.sqrt(x**2 * t - t * (self.WIDTH_ARM**2) + t**2)) / (x**2 + t))

    def pos_obstacles(self):
        """Return all 2D positions of the obstacles"""
        all_obstacles = []

        for i in list(self.world.keys()):
            if i[0:8] != "obstacle":
                continue
            x, y = self.model_pos(i)
            all_obstacles.append([x, y])
        return all_obstacles

    # def compute_path_region(self):
    #     """Return all obstacle in the path region, also the closest point (in the path region) to the tip"""
    #     pose_obj = self.model_pos('object_0')  # target object position
    #
    #
    #     # path region x axis, - self.TABLE * self.RADIUS_OBS since we can consider paralell obstacles to the target
    #     # arm_reach = pose_obj[0] - self.TABLE * self.RADIUS_OBS error
    #
    #     tip = self.tip_position()[0]
    #
    #     arm_reach = min(pose_obj[0] - self.paralell * self.RADIUS_OBS, tip + 2.5 * self.ARM_LENGTH) # add self.paralell% to able to push obstacles close to the target
    #     arm_region_minus, arm_region_plus = pose_obj[1] - self.WIDTH_ARM / \
    #         2, pose_obj[1] + self.WIDTH_ARM / 2  # path region y axis
    #     Obs_in_path_region = []
    #     Poses = self.pos_obstacles()
    #
    #     # index  in which Poses[closest_point_index] is the closest point to the tip
    #     closest_point_index = 0
    #     flag = True
    #     count = 0
    #     j = 0  # index in which Obs_in_path_region[j] is the closest point to the tip
    #     for i in range(len(Poses)):
    #         x, y = Poses[i]
    #         if arm_region_minus <= y <= arm_region_plus and tip < x < arm_reach:
    #             Obs_in_path_region.append([x, y])
    #             if flag:
    #                 closest_point_index = i
    #                 flag = False
    #             else:
    #                 count += 1
    #                 if 0 < x - tip < Poses[closest_point_index][0] - tip:
    #                     closest_point_index = i
    #                     j = count
    #
    #     return Obs_in_path_region, j

    def path_region_boundary(self):
        """Return the boundary values of the region delimited by the path_region,
        a rectangle delimited by: x: (tip, arm_reach), y: (arm_region_minus, arm_region_plus)"""

        pose_obj = self.model_pos('object_0')  # target object position
        tip = self.tip_position()[0]

        # add self.paralell% to able to push obstacles close to the target
        arm_reach = pose_obj[0] - self.paralell * self.RADIUS_OBS
        arm_region_minus, arm_region_plus = pose_obj[1] - self.WIDTH_ARM / \
            2, pose_obj[1] + self.WIDTH_ARM / 2  # path region y axis

        return tip, arm_reach, arm_region_minus, arm_region_plus

    def is_in_path_region(self, point):
        """Return True if point is in the path region"""

        tip, arm_reach, arm_region_minus, arm_region_plus = self.path_region_boundary()
        if tip < point[0] < arm_reach and arm_region_minus <= point[1] <= arm_region_plus:
            return True
        else:
            return False

    def compute_path_region(self):
        """Return all obstacle in the path region, and
        the index of closest point (in the path region) to the tip"""

        tip, arm_reach, arm_region_minus, arm_region_plus = self.path_region_boundary()

        Obs_in_path_region = []
        for i in list(self.world.keys()):
            if i[0:8] != "obstacle":
                continue
            x, y = self.model_pos(i)
            if arm_region_minus <= y <= arm_region_plus and tip < x < arm_reach:
                Obs_in_path_region.append([x, y])
        Obs_in_path_region = np.array(Obs_in_path_region)

        Obs_in_path_region.sort(axis=0)

        size = len(Obs_in_path_region)

        if size:
            sub_range = min(arm_reach, Obs_in_path_region[0, 0] + 2*self.ARM_LENGTH)

        count = size
        for i in reversed(range(size)):
            if Obs_in_path_region[i, 0] > sub_range:
                count -= 1
            else:
                break

        return Obs_in_path_region.tolist()[:count], 0

    def path_region_phi(self, phi=0):
        """Return all obstacle in the path region with phi inclination, also the closest point (in the path region) to the tip"""
        pos_obj = self.model_pos('object_0')  # target object position
        R = self.rot(-phi)
        arm_region_minus,  arm_region_plus = 0, 2 * self.WIDTH_ARM
        c, s, t = np.cos(phi), np.sin(phi), np.tan(phi)
        # path_reach before the rotation. +self.TABLE the begin of the table
        path_reach = abs(self.BOUNDARY_N / s - 2 * self.WIDTH_ARM / t)
        # print(path_reach)

        Obs_in_path_region = []
        Pos = self.pos_obstacles()  # positions of the obstales
        tip = self.trans_rot(self.tip_position(phi=phi), phi)[0]
        # print("tip", tip)

        # index  in which Pos[closest_point_index] is the closest point to the tip
        closest_point_index = 0
        flag = True
        count = 0
        j = 0  # index in which Obs_in_path_region[j] is the closest point to the tip
        for i in range(len(Pos)):
            x, y = self.trans_rot(Pos[i], phi)

            # print(arm_region_minus, y, arm_region_plus, tip, x, path_reach)
            # if arm_region_minus <= y <= arm_region_plus and tip < x < path_reach:
            if arm_region_minus < y < arm_region_plus and tip < x < path_reach:

                Obs_in_path_region.append([x, y])
                if flag:
                    closest_point_index = i
                    flag = False
                else:
                    count += 1
                    x_b, y_b = self.trans_rot(Pos[closest_point_index], phi)
                    if 0 < x - tip < x_b - tip:
                        closest_point_index = i
                        j = count
        return Obs_in_path_region, j

    def squared_CC(self, Obs, closest_pt, RADIUS_CC):
        """Return a square that contains the closest Connected Component to the tip (Circumscribed Rectangle)"""

        if not Obs:  # no obstacles
            print("no obstacles")
            return []

        N = s_neighbors(np.array(Obs), RADIUS_CC)
        x_values = []
        y_values = []
        for i in list(N.keys()):
            for j in N[i]:
                if j == closest_pt:
                    for k in N[i]:
                        x_values.append(Obs[k][0])
                        y_values.append(Obs[k][1])
                    return [[min(x_values), min(y_values)], [max(x_values), max(y_values)]]

    def move_rel_pt(self, pt_inital, pt_end, phi=0):
        """Move relative to pt_inital 2d"""
        pt_inital = np.array([pt_inital[0], pt_inital[1]])
        pt_end = np.array([pt_end[0], pt_end[1]])
        # print("\033[34m move_rel_pt: initial tip position \033[0m", tip)

        length = np.linalg.norm(pt_end - pt_inital)
        direction = (pt_end - pt_inital) / length

        print("\033[34m move to point \033[0m", pt_end)

        # print("\033[34m move_rel_pt: final tip position \033[0m", np.array(self.tip_position(phi = phi)))
        self.write_in_plan(str([direction[0], direction[1], 0])+", " + str(length))

    def action(self, a, b, c, d):
        """one action 3 moviments"""
        if self.write:
            self.write_in_plan("actions")

        self.move_rel_pt(a, b)  # move y coordinate
        self.move_rel_pt(b, c)  # move x coordinate
        # cleaning, move y coordinate
        self.move_rel_pt(c, d)

        return [a, b, c, d]

    def push_planning(self, square):
        # # reach needed to go beyond (self.RADIUS_OBS) the center point of the obstacles

        pose_obj = self.model_pos('object_0')
        tip = self.tip_position()[0]

        """ arm_region is the region where the arm can push the Connected Component away from the path region"""
        arm_region_minus, arm_region_plus = pose_obj[1] - \
            self.WIDTH_ARM / 2, pose_obj[1] + self.WIDTH_ARM / 2

        """Cannot cross the boundaries and push more than self.BOUNDARY_N - self.RADIUS_OBS (for North case, same for South)"""
        arm_region_minus = max(arm_region_minus, self.BOUNDARY_S +
                               2*self.RADIUS_OBS + self.WIDTH_ARM / 2)
        arm_region_plus = min(arm_region_plus, self.BOUNDARY_N - 2 *
                              self.RADIUS_OBS - self.WIDTH_ARM / 2)

        """square[0][0] might be bigger than the ARM_LENGHT, so select the min"""
        if square[1][0] - square[0][0] < self.ARM_LENGTH:
            max_reach = square[1][0] + self.RADIUS_OBS
            """ + self.RADIUS_OBS, because it needs to go beyond  the center point of the last obstacle"""
        else:
            # print(f"\033[99m XXX square[0][0] + self.ARM_LENGTH = {square[0][0]} + {self.ARM_LENGTH}   < square[1][0] = {square[1][0]} \033[0m")
            max_reach = square[0][0] + self.ARM_LENGTH

        # add self.paralell to be able to push obstacles close to the target
        max_reach = min(pose_obj[0] - self.paralell * self.RADIUS_OBS, max_reach)

        if max_reach - tip < 0.001:
            return print("Failed")

        if square[0][1] - arm_region_minus < 0 and arm_region_plus - square[1][1] < 0:
            print("\033[34m push connected component away \033[0m")

            """ first decide which outside region to use"""
            if pose_obj[0] < 0.3:  # < 0.3:
                out_reach = arm_region_plus + self.WIDTH_ARM / 2  # why / 4 and not 2
                clean_direction = arm_region_minus
            else:
                out_reach = arm_region_minus - self.WIDTH_ARM / 2
                clean_direction = arm_region_plus

            self.action_performed = self.action(self.tip_position(), [tip, out_reach], [
                max_reach, out_reach], [max_reach, clean_direction])

            return True

        if square[0][1] - arm_region_minus < arm_region_plus - square[1][1]:
            print("\033[34m Pushing from top to bottom \033[0m", square[0][1] -
                  arm_region_minus, "<", arm_region_plus - square[1][1])

            self.action_performed = self.action(self.tip_position(), [
                tip, square[1][1] + self.WIDTH_ARM / 2], [max_reach, square[1][1] + self.WIDTH_ARM / 2],
                [max_reach, arm_region_minus])

        else:
            print("\033[34m Pushing from bottom to top \033[0m",
                  square[0][1] - arm_region_minus, ">", arm_region_plus - square[1][1])

            self.action_performed = self.action(self.tip_position(), [tip, square[0][1] - self.WIDTH_ARM / 2], [max_reach, square[0][1] - self.WIDTH_ARM / 2],
                                                [max_reach, arm_region_plus])

        for j in range(3):
            self.action_performed[j+1] = [self.action_performed[j+1]
                                          [0], self.action_performed[j+1][1] + self.y_shift]

        return self.action_performed

    def push_planning_phi(self, square, phi=0):

        self.write_in_plan("actions")

        # # reach needed to go beyond (self.RADIUS_OBS) the center point of the obstacles
        pose_obj = self.trans_rot(self.model_pos('object_0')[0:2], phi)
        tip = self.trans_rot(self.tip_position(phi=phi), phi)[0]

        """ arm_region is the region where the arm can push the Connected Component away from the path region"""
        arm_region_minus, arm_region_plus = pose_obj[1] - \
            self.WIDTH_ARM / 2, pose_obj[1] + self.WIDTH_ARM / 2

        """Cannot cross the boundaries and push more than self.BOUNDARY_N - self.RADIUS_OBS (for North case, same for South)"""
        # arm_region_minus = max(arm_region_minus, self.BOUNDARY_S + 2*self.RADIUS_OBS + self.WIDTH_ARM / 2)
        arm_region_plus = min(arm_region_plus, self.BOUNDARY_N - 2 *
                              self.RADIUS_OBS - self.WIDTH_ARM / 2)

        """square[0][0] might be bigger than the ARM_LENGHT, so select the min"""
        if square[1][0] - square[0][0] < self.ARM_LENGTH:
            max_reach = square[1][0] + self.RADIUS_OBS
            # print("square[1][0] + self.RADIUS_OBS", max_reach)
            """ + self.RADIUS_OBS, because it needs to go beyond  the center point of the last obstacle"""
        else:
            max_reach = square[0][0] + self.ARM_LENGTH

        if max_reach - tip < 0.001:
            return print("Failed")

        if phi > 0:
            print("\033[34m Pushing from top to bottom \033[0m", phi)

            print("rotated space", [tip, square[1][1] + self.WIDTH_ARM / 2])
            self.move_rel_pt(self.tip_position(), self.rot_trans(
                [tip, square[1][1] + self.WIDTH_ARM / 2], phi), phi)  # move y coordinate

            print("rotated space", [max_reach, square[1][1] + self.WIDTH_ARM / 2])
            self.move_rel_pt(self.rot_trans(
                [tip, square[1][1] + self.WIDTH_ARM / 2], phi), self.rot_trans(
                [max_reach, square[1][1] + self.WIDTH_ARM / 2], phi), phi)

            print("rotated space", [max_reach, arm_region_minus])
            self.move_rel_pt(self.rot_trans(
                [max_reach, square[1][1] + self.WIDTH_ARM / 2], phi), self.rot_trans([max_reach, arm_region_minus], phi), phi)

        else:
            print("\033[34m Pushing for -phi \033[0m", phi)

            print("rotated space", [tip, square[0][1] - self.WIDTH_ARM / 2])
            self.move_rel_pt(self.tip_position(), self.rot_trans(
                [tip, square[0][1] - self.WIDTH_ARM / 2], phi), phi)  # move y coordinate

            print("rotated space", [max_reach, square[0][1] - self.WIDTH_ARM / 2])
            self.move_rel_pt(self.rot_trans(
                [tip, square[0][1] - self.WIDTH_ARM / 2], phi), self.rot_trans(
                [max_reach, square[0][1] - self.WIDTH_ARM / 2], phi), phi)

            print("rotated space", [max_reach, arm_region_plus])
            self.move_rel_pt(self.rot_trans(
                [max_reach, square[0][1] - self.WIDTH_ARM / 2], phi), self.rot_trans([max_reach, arm_region_plus], phi), phi)

        return True

    def write_in_plan(self, row):
        with open("plan.txt", "a") as f:
            f.write(str(row) + "\n")
