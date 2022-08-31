import os
import time
import numpy as np
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'baxter_planit', 'scripts'))
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point

"""perception and real moviments"""
from Perception.pose_estimation_v1_for_letters.get_tags import Perception
from src.real_baxter_planit.scripts.position_the_arm import position_the_arm as real_position_the_arm
from src.real_baxter_planit.scripts.get_arm_position import get_arm_position as real_get_arm_position
from src.real_baxter_planit.scripts.demo_read_plan import demo_real_plan as real_execute
# from src.baxter_planit.scripts.position_the_arm import position_the_arm as sim_position_the_arm

"""Persistent_Homology actions"""
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Persistent_Homology'))

print(os.path.abspath(__file__))
# from src.baxter_planit.scripts.PHIA_1st import PHIA_pipeline as sim_get_plan

from Persistent_Homology.PHIM import main as PHIM
from Persistent_Homology.PHIA import main as PHIA
# from src.baxter_planit.scripts.stick import spawning
# from src.baxter_planit.scripts.spawn_objects import main as spawning_object

""" Graps functions """

# from grasp import grasp_cylinder
from planit.utils import *
from planit.msg import PercievedObject
from baxter_planit import BaxterPlanner

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

""" functions that require all imported modules"""




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

    res = input('Execute? Enter')
    if res != "":
        print(res)
        print('stopped')
        return

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

    res = input('Execute? Enter')
    if res != "":
        print(res)
        print('stopped')
        return

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

    res = input('Execute? Enter')
    if res != "":
        print(res)
        print('stopped')
        return

    move_group.execute(plan, wait=True)
    move_group.stop()
    print("Displaced cylinder.")


def run_grasp(target):

    offset_x = 0
    offset_y = 0.56


    rospy.init_node("baxter_planit", anonymous=False)
    planner = BaxterPlanner(is_sim=False)

    planner.scene.add_box('table_base', list_to_pose_stamped([0.9525 + offset_x, -0.23 + offset_y, 0.3825, 0, 0, 0], 'world'), (0.81, 1.2, 0.765))
    planner.scene.add_box('table', list_to_pose_stamped([0.98 + offset_x, -0.23 + offset_y, 0.8275, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    # planner.scene.add_box('shelf_top', list_to_pose_stamped([0.98 + offset_x, -0.23 + offset_y, 0.5+0.8275, 0, 0, 0], 'world'), (0.6, 1.2, 0.205))
    planner.scene.add_box('boundaryW', list_to_pose_stamped([1.225 + offset_x, -0.265 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.09, 0.58, 0.175))
    planner.scene.add_box('boundaryS', list_to_pose_stamped([0.975 + offset_x, 0.07 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.59, 0.09, 0.175))
    planner.scene.add_box('boundaryN', list_to_pose_stamped([0.975 + offset_x, -0.60 + offset_y, 1.0575, 0, 0, 0], 'world'), (0.59, 0.09, 0.175))
    # perception_sub = rospy.Subscriber(
    #     '/perception', PercievedObject, planner.scene.updatePerception
    # )

    time.sleep(2)

    position = target
    chirality = target[4] if len(target) > 4 else 'right'
    height = float(target[5]) if len(target) > 5 else 0.235
    radius = float(target[6]) if len(target) > 6 else 0.025
    orientation = [-0.5169396533473932, 0.4994474561516194, -0.5531325099525974, 0.4211532497491772]
    input("Start?")
    grasp_cylinder(
        planner,
        height,
        radius,
        position,
        orientation,
        grasping_group=chirality + "_hand",
        group_name=chirality + "_arm",
    )
    rospy.spin()

def sim_spawn_objects():
    '''
    Input: config.txt (new object states)
    Output: objects in gazebo
    '''
    # delete current objects
    sim_delete_models()

    # # position the arm
    # sim_position_the_arm()

    # rospy.init_node('spawn_objects')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    orient = Quaternion(0, 0, 0, 1)


    path_dir = os.path.dirname(os.path.abspath(__file__))

    path_models = os.path.join(path_dir, 'models')

    f = open(os.path.join(path_models, 'cylinder/model.sdf'))

    sdff = f.read()

    obs_f = open(os.path.join(path_models, 'small_cylinder/model.sdf'))

    obs_sdff = obs_f.read()

    # TABLE = 0.68  # distance of the table to the robot at (0,0)
    # c_x = 0.255  # x coodinate of the center of the workspace

    # b_x = TABLE + c_x

    # bw_y = 0.3  # y coodinate of the center of the workspace


    position_file_address = os.path.join(path_dir, 'config.txt')
    position_file = open(position_file_address, 'r')
    obj_index = 0
    obs_index = 0
    Isobstacle = False
    ws_x, ws_y, ws_z = get_workspace_center()
    for line in position_file.readlines():
        print(line)
        if (line == "object\n"):
            continue
        elif (line == "obstacle\n"):
            Isobstacle = True
            continue
        elif Isobstacle:
            pos = line.split()
            print("throwing obstacles %d" % (obs_index))
            x = float(pos[0])
            y = float(pos[1])
            z = ws_z+0.2
            print('xyz', x,y,z)
            spawn_model('small_obstacle_'+str(obs_index), obs_sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            obs_index += 1
        else:
            pos = line.split()
            print("throwing obj %d" % (obj_index))
            x = float(pos[0])
            y = float(pos[1])
            z = ws_z+0.2
            spawn_model('object_'+str(obj_index), sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            obj_index += 1
    # rospy.signal_shutdown("Fininshed Throwing")
    f.close()
    obs_f.close()
    position_file.close()


def sim_delete_models():
    rospy.wait_for_service('/gazebo/delete_model')
    rospy.wait_for_service('/gazebo/get_world_properties')

    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

    properties = get_world_properties()
    for name in properties.model_names:
        if name[:6] == 'object':
            delete_model(name)
            print('deleting', name)
        elif name[:8] == 'obstacle':
            delete_model(name)
            print('deleting', name)


def real_perception(P:Perception):
    # cmd = 'python ./Perception/pose_estimation_v1_for_letters/get_tags.py'
    P.update_locations()

def real_calibration():
    return Perception()

def check_success(actions):
    plan_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'plan.txt'
    )
    with open(plan_file, 'r') as f:
        # each actions counts as 4 lines 
        if len(list(f.readlines())) <= 4*actions:
            return True
        else:
            return False

############################# Utils ##############################
def get_workspace_center():
    rospy.wait_for_service('/gazebo/get_model_state')

    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    pose = model_coordinates('boundaryS', 'world')
    x,z = pose.pose.position.x, pose.pose.position.z
    pose = model_coordinates('boundaryW', 'world')
    y = pose.pose.position.y
    print("x,y,z", x,y,z)
    return x,y,z

def pipeline(type_of_plan):
    actions = 0
    time_perception = 0
    time_get_arm_current_position = 0
    time_task_planning = 0
    time_success = 0
    time_motion_planning = 0
    start = time.time()
    real_position_the_arm()
    print('real_withdraw time' , time.time()-start)
    res = input('Enter')
    if res != "":
        print(res)
        print('stopped')
        return
    need_cali  = True
    while 1:
        start = time.time()
        if need_cali:
            P = real_calibration()
        real_perception(P)
        need_cali = False
        print('perception')
        print('perception time' , time.time()-start)
        time_perception = time.time()-start
        # start = time.time()
        # sim_spawn_objects()
        # print('finish spawning')
        # print('simulation withdraw time' , time.time()-start)
        start = time.time()
        real_get_arm_position()
        time_get_arm_current_position = time.time()-start
        start = time.time()
        # type_of_plan()
        time_task_planning = time.time()-start
        print('perception: ', time_perception)
        print('check arm: ', time_get_arm_current_position)
        print('task planning ', time_task_planning)
        # input('Plan ready')
        start = time.time()
        if check_success(actions):
            print('perception: ', time_perception)
            print('check arm: ', time_get_arm_current_position)
            print('task planning ', time_task_planning)
            print('END')
            break
        print('simulation plan and execute time' , time.time()-start)
        real_execute()
        actions += 1
        time_motion_planning = time.time()-start
        print('finish executing')
        print('perception: ', time_perception)
        print('check arm: ', time_get_arm_current_position)
        print('task planning ', time_task_planning)
        print('motion planning and execution: ', time_motion_planning)

        res = input('Enter')
        if res != "":
            print(res)
            print('stopped')
            return


if __name__ == '__main__':

    # P = real_calibration()
    # real_perception(P)
    # real_get_arm_position()

    # real_position_the_arm()

    # type_of_plan = sim_get_plan
    # type_of_plan = PHIA
    # pipeline(type_of_plan)

    # config_file = os.path.join(
    #     os.path.dirname(
    #         os.path.dirname(os.path.abspath(__file__))),
    #     'config.txt')

    target = [0.762, -0.244 + 0.56, 1.05]
    run_grasp(target)

    # real_execute()
    # sim_get_plan()
    # test()

    # sim_spawn_objects()
    # spawning()
    # sim_delete_models()
    # spawning_object()
    # PHIM()
    # PHIA()
