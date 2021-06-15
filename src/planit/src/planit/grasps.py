from math import pi, tau
# from moveit_commander.conversions import *
from .utils import *
from .perception import StreamedSceneInterface


class Grasps:
    def __init__(self, planning_scene_interface, filename=None):
        self.subframes = {}
        self.scene = planning_scene_interface

    def get_subframes(self, obj_db_name):
        if name in self.subframes:
            return subframes[obj_db_name]
        else:
            return list_to_pose([0, 0, 0, 0, 0, 0])

    def get_simple_grasps(self, name, ax=None, ay=None, az=None, subframe=None):
        # scene.get_known_object_names()
        # scene.get_known_object_poses()
        frame_name = name if subframe is None else name + '/' + subframe
        if subframe:
            obj = scene.get_objects([name])[name]
            absolute = pose_msg2homogeneous(obj.subframe_poses[obj.subframe_names.index(subframe)])
        else:
            absolute = pose_msg2homogeneous(self.scene.get_object_poses([name])[name])

        poses = []
        angles = (-tau / 4, 0, tau / 4, tau / 2)
        for x in angles[::2]:
            for y in angles[::2]:
                for z in angles:
                    obj_off = translation_matrix((0, 0, -0.05))
                    tab_off = translation_matrix((0, 0, 0.04))
                    relative = euler_matrix(x, y, z)
                    trans, rot = homogeneous2translation_rpy(concatenate_matrices(absolute, tab_off, relative, obj_off))
                    poses.append(trans + rot)

        return poses

    def get_simple_placements(self, pose, frame_name=None, ax=None, ay=None, az=None):
        if frame_name:
            if '/' in frame_name:
                name, frame = frame_name.split('/')[0]
                obj = scene.get_objects([name])[name]
                base = pose_msg2homogeneous(obj.subframe_poses[obj.subframe_names.index(frame)])
            else:
                base = pose_msg2homogeneous(self.scene.get_object_poses([frame_name])[name])
            absolute = concatenate_matrices(base, translation_matrix(pose))
        else:
            absolute = translation_matrix(pose)

        poses = []
        angles = (-tau / 4, 0, tau / 4, tau / 2)
        for x in angles:
            for y in angles:
                for z in angles:
                    relative = euler_matrix(x, y, z)
                    trans, rot = homogeneous2translation_rpy(concatenate_matrices(absolute, relative))
                    poses.append(trans + rot)

        return poses
