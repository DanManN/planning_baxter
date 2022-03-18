# PHIA_1st.py  2022-18-03
# MIT LICENSE 2020 Ewerton R. Vieira
# perform only the first actions

import sys
from math import pi

import time
import os

from Connected_Comp import *
import PH_planning

import numpy as np


def main():

    f = open("plan.txt", "w")

    # obj_pos_y = model_pos('object_0')[1]

    # close to wall
    # print(PH.is_close_to_wall())
    if PH.is_close_to_wall():

        print("phi", PH.phi)

        planner_timeF = time.time()

        Obs, closest_pt = PH.path_region_phi(phi=PH.phi)
        RADIUS_CC = PH.min_radius_CC(Obs)

        square = PH.squared_CC(Obs, closest_pt, RADIUS_CC)
        print("square ", square, "closest_pt ", closest_pt)

        planner_time = planner_timeF - time.time()

        PH.push_planning_phi(square, PH.phi)

        print("how close is to the goal", PH.tip_position()[0] -
              PH.model_pos('object_0')[0], "Obs set ", len(Obs), "planning time", planner_time)

    # far from wall
    else:

        planner_timeF = time.time()

        Obs, closest_pt = PH.path_region()
        RADIUS_CC = PH.min_radius_CC(Obs)

        square = PH.squared_CC(Obs, closest_pt, RADIUS_CC)
        print("square ", square, "closest_pt ", closest_pt)

        planner_time = planner_timeF - time.time()

        PH.push_planning(square)

        print("how close is to the goal", PH.tip_position()[0] -
              PH.model_pos('object_0')[0], "Obs set ", len(Obs), "planning time", planner_time)

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
