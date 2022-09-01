# PHIS.py  2022-30-04
# MIT LICENSE 2022 Ewerton R. Vieira

import sys
from math import pi

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
import time
import os

import rospkg
from util.Connected_Comp import *
import util.PH_planning as PH_planning
import util.Stick_Simulation as Stick_Simulation
import util.Tree as Tree

import numpy as np


def main(arm_length=0.2,
         radius_obs=0.039,
         width_arm=0.16,
         boundary_N=0.58,
         boundary_S=0.0,
         table=0.68,
         nu=0.015,
         h=0.08):

    Stick = Stick_Simulation.Stick_Simulation(arm_length, radius_obs, width_arm, boundary_N,
                                              boundary_S, table, nu, h)

    PH = PH_planning.PH_planning(arm_length, radius_obs, width_arm, boundary_N,
                                 boundary_S, table, nu, h, world=Stick.world())

    # override so it doesn't create a txt file and it actually moves
    PH.move_rel_pt = Stick.move_rel_tip
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
    # print("Is close to wall", PH.is_close_to_wall())
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

        planner = Tree.Tree(source_config, PH.path_region, PH.radii, PH, Stick)

        print(planner.action_list)

        Stick.write_plan(planner.action_list)

        planner_time = time.time() - t0

        print("how close is to the goal", PH.tip_position()[0] -
              PH.model_pos('object_0')[0], "Obs set ", len(PH.path_region), "planning time", planner_time)

        time.sleep(1)

        Stick.set_config(source_config)

        time.sleep(1)

        print("\033[96m execute the best actions \033[0m")

        input("press enter to execute")

        Stick.execute_plan(planner.action_list)

        Stick.set_config(source_config)


def basic_move():

    Stick = Stick_Simulation.Stick_Simulation(arm_length, radius_obs, width_arm, boundary_N,
                                              boundary_S, table, nu, h)

    success = False
    while not success:
        success = Stick.straight_movement_stick([0, 3])


if __name__ == '__main__':

    # testing theoretical example
    # RADIUS_OBS = 0.0049
    # nu = 0.00115
    # h = 0.00138
    # ARM_LENGTH = 0.05

    # basic_move()
    main()
