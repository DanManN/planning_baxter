# PHIA.py  2021-10-26
# MIT LICENSE 2020 Ewerton R. Vieira

import sys
from math import pi
from planit.msg import PercievedObject
from baxter_planit import BaxterPlanner

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
import time
import os

import rospkg
from Connected_Comp import *
import PH_planning

import numpy as np


def main():

    f = open("plan.txt", "w")

    # obj_pos_y = model_pos('object_0')[1]
    time_sim_0 = time.time()  # start timer
    time_sim = time.time() - time_sim_0  # simulation time

    count_act = 0  # number of actions made by the arm

    planner_time = 0

    planner_time0 = time.time()

    # close to wall
    print(PH.is_close_to_wall())
    if PH.is_close_to_wall():

        print("phi", PH.phi)

        time_sim_0 = time.time()  # start timer

        Obs, closest_pt = PH.path_region_phi(phi=PH.phi)
        RADIUS_CC = PH.min_radius_CC(Obs)

        # print(Obs, closest_pt)
        while len(Obs) != 0 and time_sim < 300:
            square = PH.squared_CC(Obs, closest_pt, RADIUS_CC)
            print("square ", square, "closest_pt ", closest_pt)
            planner_timeF = time.time()

            PH.push_planning_phi(square, PH.phi)

            planner_time += planner_timeF - planner_time0

            planner_time0 = time.time()

            Obs, closest_pt = PH.path_region_phi(phi=PH.phi)
            RADIUS_CC = PH.min_radius_CC(Obs)

            print("how close is to the goal", PH.tip_position(phi=PH.phi)[0] -
                  PH.model_pos('object_0')[0], "Obs set ", len(Obs))
            count_act += 1
            time_sim = time.time() - time_sim_0  # simulation time

    # far from wall
    else:

        Obs, closest_pt = PH.path_region()
        RADIUS_CC = PH.min_radius_CC(Obs)

        while len(Obs) != 0 and time_sim < 300:
            square = PH.squared_CC(Obs, closest_pt, RADIUS_CC)
            print("square ", square, "closest_pt ", closest_pt)
            planner_timeF = time.time()

            PH.push_planning(square)

            planner_time += planner_timeF - planner_time0

            planner_time0 = time.time()

            Obs, closest_pt = PH.path_region()
            RADIUS_CC = PH.min_radius_CC(Obs)

            print("how close is to the goal", PH.tip_position()[0] -
                  PH.model_pos('object_0')[0], "Obs set ", len(Obs))
            count_act += 1
            time_sim = time.time() - time_sim_0  # simulation time

        print("Number of actions = ", count_act, "\n", "Time of the simulation", time_sim)

    print("Number of actions = ", count_act, "\n", "Time of the planner",
          planner_time, "\n", "Time of the simulation", time_sim)

    print(Obs)
    PH.write_data(count_act, planner_time, time_sim, Obs, "PHIA")

    f.close()


if __name__ == '__main__':

    ARM_LENGTH = 0.2
    RADIUS_OBS = 0.03
    # RADIUS_CC = 0.1  # 0.07  # 0.315
    WIDTH_ARM = 0.12  # 0.1
    BOUNDARY_N = 0.6
    BOUNDARY_S = 0

    TABLE = 0.7
    nu = 0.015
    h = 0.08

    # nu = 0
    # h = 0

    PH = PH_planning.PH_planning(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                                 BOUNDARY_S, TABLE, nu, h)

    main()
