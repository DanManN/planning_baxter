# PHIA.py  2022-05-15
# MIT LICENSE 2020 Ewerton R. Vieira

import sys
import time
import os

from util.Connected_Comp import *
import util.PH_planning as PH_planning
import util.Stick_Simulation as Stick_Simulation
import numpy as np


def main():

    ARM_LENGTH = 0.2
    RADIUS_OBS = 0.039
    # RADIUS_CC = 0.1  # 0.07  # 0.315
    WIDTH_ARM = 0.16  # 0.12
    BOUNDARY_N = 0.58
    BOUNDARY_S = 0.0

    TABLE = 0.68  # x
    nu = 0.015
    h = 0.08



    Stick = Stick_Simulation.Stick_Simulation(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                                              BOUNDARY_S, TABLE, nu, h)


    PH = PH_planning.PH_planning(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                                 BOUNDARY_S, TABLE, nu, h, world=Stick.world())

    # override so it doesn't create a txt file and it actually moves
    PH.move_rel_pt = Stick.move_rel_tip

    source_config = Stick.read_config()

    time_sim_0 = time.time()  # start timer
    time_sim = time.time() - time_sim_0  # simulation time

    count_act = 0  # number of actions made by the arm

    planner_time = 0

    planner_time0 = time.time()

    # close to wall
    print(PH.is_close_to_wall())

    action_list = []
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

        while len(PH.path_region) != 0 and time_sim < 300:
            planner_timeF = time.time()

            square = PH.squared_CC(PH.path_region, PH.closest_pt, PH.min_radius_CC())
            print("square ", square, "closest_pt ", PH.closest_pt)

            PH.push_planning(square)

            action_list.append(PH.action_performed)

            PH.world = Stick.world()
            PH.update()

            planner_time += planner_timeF - planner_time0

            planner_time0 = time.time()

            print("how close is to the goal", PH.tip_position()[0] -
                  PH.model_pos('object_0')[0], "Obs set ", len(PH.path_region))

            count_act += 1


    Stick.write_plan(action_list)
    print("Number of actions = ", count_act, "\n", planner_time, "\n")

    time.sleep(1)

    Stick.set_config(source_config)


def first_action():

    ARM_LENGTH = 0.2
    RADIUS_OBS = 0.039
    # RADIUS_CC = 0.1  # 0.07  # 0.315
    WIDTH_ARM = 0.16  # 0.12
    BOUNDARY_N = 0.58
    BOUNDARY_S = 0.0

    TABLE = 0.68  # x

    """ The choice of parameteres (nu=0 and h=-0.1) are equivalent to compute
    the distance of all obstacles to the gripper and select
    the closest obstacle to be moved"""
    nu = 0
    h = -0.1


    Stick = Stick_Simulation.Stick_Simulation(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                                              BOUNDARY_S, TABLE, nu, h)


    PH = PH_planning.PH_planning(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                                 BOUNDARY_S, TABLE, nu, h, world=Stick.world())

    # override so it doesn't create a txt file and it actually moves
    PH.move_rel_pt = Stick.move_rel_tip

    source_config = Stick.read_config()

    planner_timeF = time.time()

    square = PH.squared_CC(PH.path_region, PH.closest_pt, PH.min_radius_CC())
    print("square ", square, "closest_pt ", PH.closest_pt)

    if square:
        PH.push_planning(square)

    else:
        print("Path region empty")
        return False

    # print(f"Stick.read_config() = {Stick.read_config()} \n self.PH.world  = {PH.world } \n self.Stick.world() = {Stick.world()}")

    planner_time = planner_timeF - time.time()

    action_list = [PH.action_performed]

    Stick.write_plan(action_list)

    print("how close is to the goal", PH.tip_position()[0] -
          PH.model_pos('object_0')[0], "Obs set ", len(PH.path_region), "planning time", planner_time)

    # time.sleep(1)
    # Stick.set_config(source_config)

    return True



if __name__ == '__main__':

    perform_action = True
    while perform_action:
        perform_action = first_action()
    # main()
