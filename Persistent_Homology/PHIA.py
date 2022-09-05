# PHIA.py  2022-05-15
# MIT LICENSE 2020 Ewerton R. Vieira

import sys
import time
import os

from util.Connected_Comp import *
import util.PH_planning as PH_planning
import util.Stick_Simulation as Stick_Simulation
import numpy as np


def main(name_plan="PHIA", arm_length=0.2,
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

    success = True
    # override so it doesn't create a txt file and it actually moves
    PH.move_rel_pt = Stick.move_rel_tip

    source_config = Stick.read_config()




    count_act = 0  # number of actions made by the arm

    time_sim = 0

    planner_time = 0

    planner_time0 = time.time()

    # close to wall
    print(PH.is_close_to_wall())

    action_list = []
    if PH.is_close_to_wall():

        """DOTO"""

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
        #     PH.push_planning_phi(square, PH.phi)
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

        while len(PH.path_region) != 0 and time_sim < 300:

            time_sim_0 = time.time()  # start timer


            square = PH.squared_CC(PH.path_region, PH.closest_pt, PH.min_radius_CC())
            print("square ", square, "closest_pt ", PH.closest_pt)

            PH.push_planning(square)

            if not Stick.is_feasible():
                print("Failure of actions")
                success = False
                break

            action_list.append(PH.action_performed)

            PH.world = Stick.world()
            PH.update()

            print("how close is to the goal", PH.tip_position()[0] -
                  PH.model_pos('object_0')[0], "Obs set ", len(PH.path_region))

            count_act += 1

            time_sim += time.time() - time_sim_0  # simulation time

    planner_time = time.time() - planner_time0


    if success:
        Stick.write_plan(action_list, name_plan=name_plan, time_to_plan=planner_time)
        print("Number of actions = ", count_act, "\n", planner_time, "\n")

    else:
        print("PLAN FAILED")

    time.sleep(1)

    Stick.set_config(source_config)


def first_action(arm_length=0.2,
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

    source_config = Stick.read_config()

    planner_timeF = time.time()

    square = PH.squared_CC(PH.path_region, PH.closest_pt, PH.min_radius_CC())
    print("square ", square, "closest_pt ", PH.closest_pt)

    if square:
        PH.push_planning(square)

    else:
        print("Path region empty")

    # print(f"Stick.read_config() = {Stick.read_config()} \n self.PH.world  = {PH.world } \n self.Stick.world() = {Stick.world()}")

    planner_time = planner_timeF - time.time()

    action_list = [PH.action_performed]

    Stick.write_plan(action_list, name_plan=name_plan, time_to_plan=planner_time)

    print("how close is to the goal", PH.tip_position()[0] -
          PH.model_pos('object_0')[0], "Obs set ", len(PH.path_region), "planning time", planner_time)

    time.sleep(1)
    Stick.set_config(source_config)


if __name__ == '__main__':

    # default constants
    ARM_LENGTH = 0.2
    RADIUS_OBS = 0.039
    # RADIUS_CC = 0.1  # 0.07  # 0.315
    WIDTH_ARM = 0.16  # 0.12
    BOUNDARY_N = 0.58
    BOUNDARY_S = 0.0

    TABLE = 0.68  # x
    NU = 0.015
    H = 0.08

    # first_action()
    name_plan = os.path.splitext(os.path.basename(__file__))[0]
    main()
