#!/usr/bin/env python

import sys
import time
import rospy
from math import pi
from planit.msg import PercievedObject
from baxter_planit import BaxterPlanner
import numpy as np
import PH_planning


app_dist = 0.1
WIDTH_ARM = 0.12  # diameter of the cylinder for the wrist


def position_the_arm():

    planner = BaxterPlanner(False)

    y = PH.model_pos('object_0')[1]
    BOUNDARY_N = 0.6
    phi = 0
    WIDTH_ARM = 0.12
    z = PH.link_pos()[2]

    flag = 1  # 0 to repete again
    # while np.linalg.norm(np.array([0.72, 1.13]) - np.array([tip_position(phi=phi)[0], z])) > 0.25 or flag < 2:
    while flag < 2:
        flag += 1
        planner.do_end_effector('close')
        plan, planning_time = planner.plan_line_traj([-1, 0, 1], 0.3)
        planner.execute(plan, v_scale=1)

        if y > BOUNDARY_N - WIDTH_ARM:
            phi = PH.angle_phi()
            success, plan, planning_time, error_code = planner.plan_ee_pose(
                [0.72, WIDTH_ARM / np.cos(phi), 1.13, -pi / 2 - phi, pi / 2, -pi / 2])
            # print(success, planning_time, error_code)
            if not success:
                return False
            planner.execute(plan, v_scale=1)
        elif y < WIDTH_ARM:
            phi = angle_phi()
            success, plan, planning_time, error_code = planner.plan_ee_pose(
                [0.72, BOUNDARY_N - WIDTH_ARM / np.cos(phi), 1.13, -pi / 2 + phi, pi / 2, -pi / 2])
            # print(success, planning_time, error_code)
            if not success:
                return False
            planner.execute(plan, v_scale=1)
        else:
            success, plan, planning_time, error_code = planner.plan_ee_pose(
                [0.72, 0.3, 1.13, -pi / 2, -pi / 2, -pi / 2])
            # [0.82, 0.25, 1.33, pi/4, +pi / 2, -pi / 2])
            # print(success, planning_time, error_code)
            if not success:
                return False
            planner.execute(plan, v_scale=1)

    print(PH.angle_phi())


if __name__ == '__main__':
    rospy.init_node("baxter_planit", anonymous=False)

    ARM_LENGTH = 0.2
    RADIUS_OBS = 0.03
    # RADIUS_CC = 0.1
    WIDTH_ARM = 0.12
    BOUNDARY_N = 0.6
    BOUNDARY_S = 0

    TABLE = 0.7
    nu = 0.015
    h = 0.08

    PH = PH_planning.PH_planning(ARM_LENGTH, RADIUS_OBS, WIDTH_ARM, BOUNDARY_N,
                                 BOUNDARY_S, TABLE, nu, h)
    position_the_arm()
