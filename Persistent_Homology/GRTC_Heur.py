# GRTC_Heuristic.py  2022-05-15
# MIT LICENSE 2020 Ewerton R. Vieira

import sys
import time
import os

from util.Connected_Comp import *
import util.PH_planning as PH_planning
import util.Stick_Simulation as Stick_Simulation
import numpy as np
import random


def points_out_PR(PH):
    """sample points outsite path region"""

    x_o, y_o = PH.path_region[PH.closest_pt]
    x_g, y_g, _ = PH.model_pos('object_0')

    all_obj = PH.pos_obstacles()

    x = np.linspace(PH.TABLE + PH.RADIUS_OBS, x_g, int((x_g - (PH.TABLE + PH.RADIUS_OBS))/0.03))
    y = np.linspace(PH.RADIUS_OBS, 0.58 - PH.RADIUS_OBS, int((0.58 - 2*PH.RADIUS_OBS)/0.03))
    # print("x", x)
    # print("y", y)

    xx, yy = np.meshgrid(x, y)
    # print("xx", xx)
    # print("yy", yy)
    # print("shape", xx.shape)
    # print("shape", yy.shape)
    L_near = []
    L_far = []
    for j in range(xx.shape[1]):
        for i in range(yy.shape[0]):
            flag = True
            x_, y_ = xx[i, j], yy[i, j]
            if PH.is_in_path_region([x_, y_]):
                continue

            for point in all_obj:
                if np.linalg.norm(np.array([x_ - point[0], y_ - point[1]])) < 0.07:
                    flag = False
                    break
            if flag:
                if np.linalg.norm(np.array([x_ - x_o, y_ - y_o])) < 0.20:
                    L_near.append([x_, y_])
                else:
                    L_far.append([x_, y_])
    random.shuffle(L_near)
    random.shuffle(L_far)
    return L_near + L_far


def move_relative_tip(Stick, point):
    tip = Stick.tip_position()
    success = Stick.move_rel_tip([tip, point])
    return success, Stick.points2direction([tip, point])


def action(PH, Stick):

    x_o, y_o = PH.path_region[PH.closest_pt]
    tip = Stick.tip_position()

    success, act1 = move_relative_tip(Stick, [tip[0], y_o + sign * 1.2 * PH.RADIUS_OBS]])
    if not success:
        return False
    tip= Stick.tip_position()
    success, act2= move_relative_tip(Stick, [x_o, tip[1]]])
    if not success:
        return False
    tip= Stick.tip_position()
    success, act3= move_relative_tip(Stick, point])
    if not success:
        return False
    tip= Stick.tip_position()
    success, act4= move_relative_tip(Stick, [x_o, y_o])
    if not success:
        return False

    return act1+act2+act3+act4



def try_actions(PH, Stick):

    List = points_out_PR(PH)

    config_temp= Stick.read_config()

    while len(List) != 0:
        point = List[0]
        read_action= action(PH, Stick, point)
        if read_action:
            break
        List.pop(0)

        Stick.set_config(config_temp)  # reset config

        PH.world = Stick.world()
        PH.update()

    if read_action:
        return read_action
    return False




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

    source_config = Stick.read_config()

    time_sim_0 = time.time()  # start timer
    time_sim = time.time() - time_sim_0  # simulation time

    count_act = 0  # number of actions made by the arm

    planner_time = 0

    planner_time0 = time.time()

    action_list = []

    while len(PH.path_region) != 0 and time_sim < 300:
        planner_timeF = time.time()

        read_action= try_actions(PH, Stick)
        if not read_action:
            print("Plan Failure")
            break

        action_list.append(read_action)


        PH.world = Stick.world()
        PH.update()

        planner_time += planner_timeF - planner_time0

        planner_time0 = time.time()

        print("how close is to the goal", PH.tip_position()[0] -
              PH.model_pos('object_0')[0], "Obs set ", len(PH.path_region))

        count_act += 1

    Stick.write_plan(action_list, number_of_acts=4)
    print("Number of actions = ", count_act, "\n", planner_time, "\n")

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
    main()
