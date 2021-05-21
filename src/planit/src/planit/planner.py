import sys
import copy
from math import pi, tau, dist, fabs, cos

import rospy
import moveit_msgs.msg
# import geometry_msgs.msg
import baxter_core_msgs.msg

import moveit_commander
from moveit_commander.conversions import *


class Planner:
    def __init__(self, is_sim=True, commander_args=[]):
        moveit_commander.roscpp_initialize(commander_args)
        # rospy.init_node("moveit_pyplanning", anonymous=False)

        self.is_sim = is_sim
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

    def testScene(self):
        self.scene.add_box('cafe_table.link', list_to_pose_stamped([1.0, 0.5, 0.78, 0, 0, 0], 'world'), (0.2, 0.4, 0.4))
        self.scene.add_box(
            'cafe_table_clone.link', list_to_pose_stamped([0.5, 1.0, 0.78, 0, 0, 0], 'world'), (0.4, 0.2, 0.4)
        )
        self.scene.add_box(
            'wood_block_10_2_1cm.link', list_to_pose_stamped([1.0, 0.5, 1.03, 0, 0, 0], 'world'), (0.01, 0.02, 0.1)
        )

    def do_end_effector(self, command, group_name="left_hand"):
        pass
        ### example fake implementation ###
        # move_group = moveit_commander.MoveGroupCommander(group_name)
        # joint_goal = move_group.get_current_joint_values()

        # if command == 'open':
        #     joint_goal = [0.02, -0.02]
        # elif command == 'close':
        #     joint_goal = [0.0, 0.0]

        # move_group.go(joint_goal, wait=True)
        # move_group.stop()

    def plan_ee_pose(self, ee_pose=[0, 0, 0, 0, 0, 0], group_name="left_arm"):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_num_planning_attempts(50)
        move_group.set_planning_time(1.0)
        # move_group.set_planner_id("SPARSkConfigDefault")
        # move_group.set_planner_id("LazyPRMstarkConfigDefault")
        move_group.set_planner_id("PersistentLazyPRMstarLoad")
        # move_group.set_support_surface_name(support)

        move_group.set_pose_target(ee_pose)
        plan = move_group.plan()
        move_group.clear_pose_targets()

        return plan

    def plan_ee_poses(self, ee_poses=[[0, 0, 0, 0, 0, 0]], group_name="left_arm"):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        # move_group.set_support_surface_name(support)

        move_group.set_pose_targets(ee_poses)
        plan = move_group.plan()
        move_group.clear_pose_targets()

        return plan

    def plan_line_traj(
        self, direction=[0, 0, 1], magnitude=1, eef_step=0.005, jump_threshold=0.0, group_name="left_arm"
    ):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        # move_group.set_support_surface_name(support)
        scale = magnitude / dist(direction, (0, 0, 0))

        wpose = move_group.get_current_pose().pose
        wpose.position.x += scale * direction[0]
        wpose.position.y += scale * direction[1]
        wpose.position.z += scale * direction[2]
        waypoints = [copy.deepcopy(wpose)]

        plan, fraction = move_group.compute_cartesian_path(
            waypoints,
            eef_step,
            jump_threshold,
            # avoid_collisions=False
        )

        return plan, fraction

    def execute(self, raw_plan, v_scale=0.25, a_scale=1.0, group_name="left_arm"):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        plan = move_group.retime_trajectory(
            self.robot.get_current_state(),
            raw_plan,
            velocity_scaling_factor=v_scale,
            acceleration_scaling_factor=a_scale,
        )
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.execute(plan, wait=True)
        move_group.stop()

    def wait_for_state_update(self, obj_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([obj_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = obj_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def attach(self, obj_name, grasping_group='left_hand', group_name="left_arm", timeout=4):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        eef_link = move_group.get_end_effector_link()

        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(eef_link, obj_name, touch_links=touch_links)

        return self.wait_for_state_update(obj_name, box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach(self, obj_name, group_name="left_arm", timeout=4):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        eef_link = move_group.get_end_effector_link()

        self.scene.remove_attached_object(eef_link, name=obj_name)

        return self.wait_for_state_update(obj_name, box_is_known=True, box_is_attached=False, timeout=timeout)
