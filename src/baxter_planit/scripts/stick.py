import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point
import os
from math import pi
import tf

rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')

def spawning():
    # rospy.init_node('spawn_objects')


    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    def q_from_e(r,p,y):
        return tf.transformations.quaternion_from_euler(r, p, y)

    orient = Quaternion(0, -0.7071067811865475, 0, 0.7071067811865475)

    path_dir = os.path.dirname(__file__)

    path_models = os.path.join(path_dir, '../models')

    f = open(os.path.join(path_models, 'stick/model.sdf'))


    # f = open('/home/pracsys/retrieval/Kai/Aggregation/models/stick/model.sdf')


    # f = open(os.path.join(path_dir, '../../../' , 'models', 'stick', 'model.sdf'))
    #
    # print(q_from_e(pi / 2, -pi / 2, -pi / 2))
    # a, b, c, d = q_from_e(pi / 2, -pi / 2, -pi / 2)
    # orient = Quaternion(a, b, c, d)
    # print(orient)

    sdff = f.read()

    lengh_gripper2elbow = 0.3575

    x, y, z = 0.7 - lengh_gripper2elbow, -0.3, 1.13

    spawn_model('stick', sdff, "",
                Pose(Point(x=x, y=y, z=z), orient), "world")

    f.close()


if __name__ == '__main__':
    spawning()
