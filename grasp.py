#!/usr/bin/env python
import sys
import time
import copy
from math import pi, tau, dist

import numpy as np
import pybullet as p

import rospy
import std_msgs
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import *

from planit.utils import *
from planit.msg import PercievedObject
from baxter_planit import BaxterPlanner


def grasp_cylinder(
    planner,
    height,
    radius,
    position,
    orientation=[0, 0, 0, 1],
    offset=(0.0, 0.0, 0.01),
    gripper_width=0.042,
    resolution=8,
    pre_disp_dist=0.1,
    post_disp_dir=(0, 0, 1),
    post_disp_dist=0.2,
    eef_step=0.005,
    jump_threshold=0.0,
    v_scale=0.25,
    a_scale=1.0,
    grasping_group="left_hand",
    group_name="left_arm",
):
    """
    resolution must be even
    """

    res = resolution + 1
    hres = (res // 2) + 1

    # grasp orientations
    # vertical
    # vert = []
    # x = 0
    # y = pi
    # # rotate along gripper axis:
    # for z in np.linspace(-pi, pi, res):
    #     vert.append(p.getQuaternionFromEuler((x, y, z)))

    # horizontal
    horz = []
    x = -pi / 2
    # rotate along gripper axis:
    for y in np.linspace(-pi, pi, res):
        # rotate along horizontal axis:
        for z in np.linspace(-pi, 0, hres):
            horz.append(p.getQuaternionFromEuler((x, y, z)))

    # object position and orientation
    obj_pos, obj_rot = position, orientation
    gw = gripper_width  # gripper width

    # positions along shape
    grasps = []
    h, r = height, radius
    # for z in np.linspace(0.0, 0.3, hres):
    #     grasps += [[(0, 0, z * h), o] for o in vert]
    for z in np.linspace(-0.1, 0.3, hres):
        grasps += [
            [(0, 0, z * h), o]
            for o in horz[hres * ((res - 1) // 4):hres * ((res + 3) // 4)]
        ]
        grasps += [
            [(0, 0, z * h), o]
            for o in horz[hres * ((-res - 1) // 4):hres * ((-res + 3) // 4)]
        ]
    offset = (offset[0], offset[1], offset[2] + r / 2 - pre_disp_dist)

    poses = []
    for pos, rot in grasps:
        pos_inv, rot_inv = p.invertTransform(pos, rot)
        off, roff = p.multiplyTransforms((0, 0, 0), rot, offset, rot_inv)
        n_pos, n_rot = p.multiplyTransforms(off, roff, pos, rot)
        tpos, trot = p.multiplyTransforms(obj_pos, obj_rot, n_pos, n_rot)
        pose = list(tpos) + list(trot)
        poses.append(pose)

    # set up planner for move_group
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_num_planning_attempts(25)
    move_group.set_planning_time(10.0)

    # open gripper
    planner.do_end_effector('open', group_name=grasping_group)

    # plan to pre goal poses
    move_group.set_pose_targets(poses)
    success, raw_plan, planning_time, error_code = move_group.plan()
    move_group.clear_pose_targets()
    if not success:
        return error_code
    else:
        print("Planned pick for cylinder in", planning_time, "seconds.")

    # retime and execute trajectory
    plan = move_group.retime_trajectory(
        planner.robot.get_current_state(),
        raw_plan,
        velocity_scaling_factor=v_scale,
        acceleration_scaling_factor=a_scale,
    )
    move_group.execute(plan, wait=True)
    move_group.stop()

    # slide to goal
    cpose = pose_msg2homogeneous(move_group.get_current_pose().pose)
    trans = translation_matrix((0, 0, pre_disp_dist))
    wpose = homogeneous2pose_msg(concatenate_matrices(cpose, trans))
    waypoints = [copy.deepcopy(wpose)]
    raw_plan, fraction = move_group.compute_cartesian_path(
        waypoints, eef_step, jump_threshold, avoid_collisions=False
    )
    print("Planned approach", fraction * pre_disp_dist, "for cylinder.")
    if fraction < 0.5:
        return -fraction

    # retime and execute trajectory
    plan = move_group.retime_trajectory(
        planner.robot.get_current_state(),
        raw_plan,
        velocity_scaling_factor=v_scale,
        acceleration_scaling_factor=a_scale,
    )
    move_group.execute(plan, wait=True)
    move_group.stop()
    print("Approached cylinder.")

    # close gripper
    planner.do_end_effector('close', group_name=grasping_group)

    # attach to robot chain
    # success = planner.attach(obj_name, grasping_group=grasping_group, group_name=group_name)
    # if success:
    #     print("Picked", obj_name, ".")
    # else:
    #     return -1

    # displace
    scale = post_disp_dist / dist(post_disp_dir, (0, 0, 0))
    wpose = move_group.get_current_pose().pose
    wpose.position.x += scale * post_disp_dir[0]
    wpose.position.y += scale * post_disp_dir[1]
    wpose.position.z += scale * post_disp_dir[2]
    waypoints = [copy.deepcopy(wpose)]
    raw_plan, fraction = move_group.compute_cartesian_path(
        waypoints, eef_step, jump_threshold, avoid_collisions=False
    )
    print("Planned displacement", fraction * post_disp_dist, "for cylinder.")
    if fraction < 0.5:
        return -fraction

    # retime and execute trajectory
    plan = move_group.retime_trajectory(
        planner.robot.get_current_state(),
        raw_plan,
        velocity_scaling_factor=v_scale,
        acceleration_scaling_factor=a_scale,
    )
    move_group.execute(plan, wait=True)
    move_group.stop()
    print("Displaced cylinder.")


if __name__ == '__main__':
    offset_x = 0
    offset_y = 0.56

    if len(sys.argv) <= 3:
        print("Error:", sys.argv[0], "specifiy position! <x> <y> <z>")
        sys.exit(-1)
    rospy.init_node("baxter_planit", anonymous=False)
    planner = BaxterPlanner(is_sim=False)
    planner.scene.add_box('table_base', list_to_pose_stamped([0.9525 + offset_x, -0.23 + offset_y, 0.3825, 0, 0, 0], 'world'), (0.81, 1.2, 0.765))
    planner.scene.add_box('table', list_to_pose_stamped([0.98 + offset_x, -0.23 + offset_y, 0.8275, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    planner.scene.add_box('shelf_top', list_to_pose_stamped([0.98 + offset_x, -0.23 + offset_y, 0.5+0.8275, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    planner.scene.add_box('boundaryW', list_to_pose_stamped([1.225 + offset_x, -0.265 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.09, 0.58, 0.175))
    planner.scene.add_box('boundaryS', list_to_pose_stamped([0.975 + offset_x, 0.07 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.59, 0.09, 0.175))
    planner.scene.add_box('boundaryN', list_to_pose_stamped([0.975 + offset_x, -0.60 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.59, 0.09, 0.175))
    # perception_sub = rospy.Subscriber(
    #     '/perception', PercievedObject, planner.scene.updatePerception
    # )
    time.sleep(2)

    position = [float(sys.argv[i]) for i in range(1, 4)]
    chirality = sys.argv[4] if len(sys.argv) > 4 else 'right'
    height = float(sys.argv[5]) if len(sys.argv) > 5 else 0.235
    radius = float(sys.argv[6]) if len(sys.argv) > 6 else 0.025
    input("Start?")
    grasp_cylinder(
        planner,
        height,
        radius,
        position,
        grasping_group=chirality + "_hand",
        group_name=chirality + "_arm",
    )
    rospy.spin()
