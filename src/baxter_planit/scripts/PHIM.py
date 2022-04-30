# PHIM.py  2022-30-04
# MIT LICENSE 2022 Ewerton R. Vieira

import sys
from math import pi

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
import time
import os

import rospkg
from Connected_Comp import *
import PH_planning
import Stick_Simulation
import MCTS

import numpy as np


def main():
    #
    # # obj_pos_y = model_pos('object_0')[1]
    # time_sim_0 = time.time()  # start timer
    # time_sim = time.time() - time_sim_0  # simulation time
    #
    # count_act = 0  # number of actions made by the arm
    #
    planner_time = 0

    t0 = time.time()

    # close to wall
    print(PH.is_close_to_wall())
    if PH.is_close_to_wall():

        print("too close to wall")

        # print("phi", PH.phi)
        #
        # time_sim_0 = time.time()  # start timer
        #
        # Obs, closest_pt = PH.path_region_phi(phi=PH.phi)
        # RADIUS_CC = PH.min_radius_CC(Obs)
        #
        # # print(Obs, closest_pt)
        # while len(Obs) != 0 and time_sim < 300:
        #     square = PH.squared_CC(Obs, closest_pt, RADIUS_CC)
        #     print("square ", square, "closest_pt ", closest_pt)
        #     planner_timeF = time.time()
        #
        #     Stick.push_planning_phi(square, PH.phi)
        #
        #     planner_time += planner_timeF - planner_time0
        #
        #     planner_time0 = time.time()
        #
        #     Obs, closest_pt = PH.path_region_phi(phi=PH.phi)
        #     RADIUS_CC = PH.min_radius_CC(Obs)
        #
        #     print("how close is to the goal", PH.tip_position(phi=PH.phi)[0] -
        #           PH.model_pos('object_0')[0], "Obs set ", len(Obs))
        #     count_act += 1
        #     time_sim = time.time() - time_sim_0  # simulation time

    # far from wall
    else:

        source_config = Stick.read_config()

        planner = MCTS.MCTS(source_config, PH.path_region, PH.radii, PH, Stick)

        print(planner.action_list)

        write_plan(planner.action_list)

        planner_time = time.time() - t0

        print("how close is to the goal", PH.tip_position()[0] -
              PH.model_pos('object_0')[0], "Obs set ", len(PH.path_region), "planning time", planner_time)

        time.sleep(1)

        Stick.set_config(source_config)


def points2direction(pt_inital, pt_end):
    pt_inital = np.array([pt_inital[0], pt_inital[1]])
    pt_end = np.array([pt_end[0], pt_end[1]])
    length = np.linalg.norm(pt_end - pt_inital)
    direction = (pt_end - pt_inital) / length
    return str([direction[0], direction[1], 0])+", " + str(length)


def write_plan(action_list):

    f = open("plan.txt", "w")
    for action in action_list:
        f.write("actions\n")
        for j in range(3):
            f.write(points2direction(action[j], action[j+1]) + "\n")
    f.close()


def basic_move():

    Stick.straight_movement_stick([0.88, -0.05])


def first_action():

    config = Stick.read_config()

    planner_timeF = time.time()

    square = PH.squared_CC(PH.path_region, PH.closest_pt, PH.min_radius_CC())
    print("square ", square, "closest_pt ", PH.closest_pt)

    if square:
        PH.push_planning(square)

    else:
        print("Path region empty")

    # print(f"Stick.read_config() = {Stick.read_config()} \n self.PH.world  = {PH.world } \n self.Stick.world() = {Stick.world()}")

    planner_time = planner_timeF - time.time()

    print("how close is to the goal", PH.tip_position()[0] -
          PH.model_pos('object_0')[0], "Obs set ", len(PH.path_region), "planning time", planner_time)

    time.sleep(1)
    Stick.set_config(config)


if __name__ == '__main__':

    ARM_LENGTH = 0.2
    RADIUS_OBS = 0.039
    # RADIUS_CC = 0.1  # 0.07  # 0.315
    WIDTH_ARM = 0.16  # 0.12
    BOUNDARY_N = 0.58
    BOUNDARY_S = 0.0

    TABLE = 0.68  # x
    nu = 0.015
    h = 0.08

    # testing theoretical example
    # RADIUS_OBS = 0.0049
    # nu = 0.00115
    # h = 0.00138
    # ARM_LENGTH = 0.05

    Stick = Stick_Simulation.Stick_Simulation(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                                              BOUNDARY_S, TABLE, nu, h)

    PH = PH_planning.PH_planning(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                                 BOUNDARY_S, TABLE, nu, h, world=Stick.world())

    # override so it doesn't create a txt file and it actually moves
    PH.move_rel_pt = Stick.move_rel_tip

    # first_action()
    # basic_move()
    main()
