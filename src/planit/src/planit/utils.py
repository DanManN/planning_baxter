from tf.transformations import *
from geometry_msgs.msg import Pose


def homogeneous2translation_quaternion(homogeneous):
    """
  Translation: [x, y, z]
  Quaternion: [x, y, z, w]
  """
    trans = list(translation_from_matrix(homogeneous))
    quat = list(quaternion_from_matrix(homogeneous))
    return trans, quat


def homogeneous2translation_rpy(homogeneous):
    """
  Translation: [x, y, z]
  RPY: [sx, sy, sz]
  """
    trans = list(translation_from_matrix(homogeneous))
    rpy = list(euler_from_matrix(homogeneous))
    return trans, rpy


def homogeneous2pose_msg(homogeneous):
    pose = Pose()
    trans, quat = homogeneous2translation_quaternion(homogeneous)
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def pose_msg2homogeneous(pose):
    trans = translation_matrix((pose.position.x, pose.position.y, pose.position.z))
    rot = quaternion_matrix(
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    )
    return concatenate_matrices(trans, rot)


def pose_msg2list(pose):
    trans = (pose.position.x, pose.position.y, pose.position.z)
    rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return trans, rot


def sgn(x):
    return x / abs(x)
