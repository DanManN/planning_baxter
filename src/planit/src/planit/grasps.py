from math import pi, tau
import numpy as np
# from moveit_commander.conversions import *
import pybullet as p
from shape_msgs.msg import SolidPrimitive as SP

from .utils import *
from .perception import StreamedSceneInterface


class Grasps:

    def __init__(self, planning_scene_interface, gripper_width, filename=None):
        self.subframes = {}
        self.scene = planning_scene_interface
        self.gripper_width = gripper_width

    def get_subframes(self, obj_db_name):
        if name in self.subframes:
            return subframes[obj_db_name]
        else:
            return list_to_pose([0, 0, 0, 0, 0, 0])

    def get_simple_grasps(self, name, offset=(0, 0, 0), resolution=8):
        """
        resolution must be even
        """
        obj = self.scene.get_objects([name])[name]

        shape = obj.primitives[0]
        res = 5 if shape.type == SP.BOX else resolution + 1
        hres = (res // 2) + 1

        # grasp orientations
        # vertical
        vert = []
        x = 0
        y = pi
        # rotate along gripper axis:
        for z in np.linspace(-pi, pi, res):
            vert.append(p.getQuaternionFromEuler((x, y, z)))

        # horizontal
        horz = []
        x = -pi / 2
        # rotate along gripper axis:
        for y in np.linspace(-pi, pi, res):
            # rotate along horizontal axis:
            for z in np.linspace(-pi, 0, hres):
                horz.append(p.getQuaternionFromEuler((x, y, z)))

        # object position and orientation
        obj_pos, obj_rot = pose_msg2list(obj.pose)

        def nearOdd(n):
            return round((n - 1) / 2) * 2 + 1

        # positions along shape
        grasps = []
        if shape.type == SP.BOX:
            sx, sy, sz = shape.dimensions

            # gw = self.right_flim[1] * 2  # gripper width
            gw = self.gripper_width  # gripper width

            # top = [0, 0, sz / 2]
            # left = [0, -sy / 2, 0]
            # right = [0, sy / 2, 0]
            # front = [-sx / 2, 0, 0]
            if sx < gw:
                noz = nearOdd(sz / (gw * 1.5))
                for z in np.linspace(-(noz - 1) / (2 * noz), (noz - 1) / (2 * noz), noz):
                    grasps.append([[0, sy / 2, z * sz], horz[3]])  # right
                    grasps.append([[0, sy / 2, z * sz], horz[9]])  # right
                    grasps.append([[0, -sy / 2, z * sz], horz[5]])  # left
                    grasps.append([[0, -sy / 2, z * sz], horz[11]])  # left
                noy = nearOdd(sy / gw)
                for y in np.linspace(-(noy - 1) / (2 * noy), (noy - 1) / (2 * noy), noy):
                    grasps.append([[0, y * sy, sz / 2], vert[1]])  # top
                    grasps.append([[0, y * sy, sz / 2], vert[3]])  # top
            if sy < gw:
                noz = nearOdd(sz / gw)
                for z in np.linspace(-(noz - 1) / (2 * noz), (noz - 1) / (2 * noz), noz):
                    grasps.append([[-sx / 2, 0, z * sz], horz[4]])  # front
                    grasps.append([[-sx / 2, 0, z * sz], horz[10]])  # front
                nox = nearOdd(sx / gw)
                for x in np.linspace(-(nox - 1) / (2 * nox), (nox - 1) / (2 * nox), nox):
                    grasps.append([[x * sx, 0, sz / 2], vert[0]])  # top
                    grasps.append([[x * sx, 0, sz / 2], vert[2]])  # top
        elif shape.type == SP.CYLINDER or shape.type == SP.CONE:
            h, r = shape.dimensions
            noz = nearOdd(h / (gw))
            for z in np.linspace(-(noz - 1) / (2 * noz), (noz - 1) / (2 * noz), noz):
                grasps += [[(0, 0, z * h), o] for o in vert]
                grasps += [
                    [(0, 0, z * h), o]
                    for o in horz[hres * ((res - 1) // 4):hres * ((res + 3) // 4)]
                ]
                grasps += [
                    [(0, 0, z * h), o]
                    for o in horz[hres * ((-res - 1) // 4):hres * ((-res + 3) // 4)]
                ]
                offset = (0, 0, 0)
        elif shape.type == SP.SPHERE:
            r = shape.dimensions[0]
            grasps = [[(0, 0, 0), o] for o in vert + horz]
        # elif shape[2] == p.GEOM_MESH:
        # elif shape[2] == p.GEOM_PLANE:

        # adjust offset for finger width
        offset = (offset[0], offset[1], offset[2] + self.gripper_width / 2)

        poses = []
        for pos, rot in grasps:
            pos_inv, rot_inv = p.invertTransform(pos, rot)
            off, roff = p.multiplyTransforms((0, 0, 0), rot, offset, rot_inv)
            n_pos, n_rot = p.multiplyTransforms(off, roff, pos, rot)
            tpos, trot = p.multiplyTransforms(obj_pos, obj_rot, n_pos, n_rot)
            pose = [tpos, trot]
            poses.append(pose)

        return poses

    def get_simple_placements(self, obj_name, pose, abs_off=None, frame_name=None):
        if frame_name:
            if '/' in frame_name:
                name, frame = frame_name.split('/')[0]
                obj = scene.get_objects([name])[name]
                base = pose_msg2homogeneous(
                    obj.subframe_poses[obj.subframe_names.index(frame)]
                )
            else:
                base = pose_msg2homogeneous(
                    self.scene.get_object_poses([frame_name])[name]
                )
            absolute = concatenate_matrices(base, translation_matrix(pose))
        else:
            absolute = translation_matrix(pose)

        if abs_off:
            offset = translation_matrix(abs_off)
        else:
            offset = identity_matrix()

        poses = []
        angles = (-tau / 4, 0, tau / 4, tau / 2)
        for x in angles:
            for y in angles:
                for z in angles:
                    relative = euler_matrix(x, y, z)
                    trans, rot = homogeneous2translation_rpy(
                        concatenate_matrices(absolute, offset, relative)
                    )
                    poses.append(trans + rot)

        return poses
