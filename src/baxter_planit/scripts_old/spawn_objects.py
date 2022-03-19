import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point
import os
import copy

from std_srvs.srv import Empty
pause_physics_client=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
unpause_physics_client=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)

rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')

get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_model_coordinates = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    

def get_workspace_center():
    pose = model_coordinates('boundaryS', 'world')
    x,z = pose.pose.position.x, pose.pose.position.z
    pose = model_coordinates('boundaryW', 'world')
    y = pose.pose.position.y
    print("x,y,z", x,y,z)
    return x,y,z

def spawn_objects():
    # rospy.init_node('spawn_objects')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    orient = Quaternion(0, 0, 0, 1)


    path_dir = os.path.dirname(__file__)

    path_models = os.path.join(path_dir, '../../../models')

    f = open(os.path.join(path_models, 'target_cup/model.sdf'))

    sdff = f.read()

    obs_f = open(os.path.join(path_models, 'cup/model.sdf'))

    obs_sdff = obs_f.read()

    # TABLE = 0.68  # distance of the table to the robot at (0,0)
    # c_x = 0.255  # x coodinate of the center of the workspace

    # b_x = TABLE + c_x

    # bw_y = 0.3  # y coodinate of the center of the workspace


    position_file_address = os.path.join(path_dir, 'Examples', 'simple_1.txt')
    position_file = open(position_file_address, 'r')
    obj_index = 0
    obs_index = 0
    Isobstacle = False
    ws_x, ws_y, ws_z = get_workspace_center()
    for line in position_file.readlines():
        print(line)
        if (line == "objects\n"):
            continue
        elif (line == "obstacles\n"):
            Isobstacle = True
            continue
        elif Isobstacle:
            pos = line.split()
            print("throwing obstacles %d" % (obs_index))
            x = float(pos[0])
            y = ws_y + float(pos[1])
            z = ws_z+0.2
            spawn_model('small_obstacle_'+str(obs_index), obs_sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            obs_index += 1
        else:
            pos = line.split()
            print("throwing obj %d" % (obj_index))
            x = float(pos[0])
            y = ws_y + float(pos[1])
            z = ws_z+0.2
            spawn_model('object_'+str(obj_index), sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            obj_index += 1
    # rospy.signal_shutdown("Fininshed Throwing")
    f.close()
    obs_f.close
    position_file.close()
    offset_x = 0
    # rospy.wait_for_service('/gazebo/get_model_state')
    # model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    pose = model_coordinates('boundaryN', 'world')
    offset_y = - pose.pose.position.y
    shift_models((offset_x, offset_y))


def shift_models(offset):
    pause_physics_client()
    print('offset', offset)
    properties = get_world_properties()
    for name in properties.model_names:
        if name == 'baxter':
            continue
        pose = model_coordinates(name, 'world')
        new_pose = copy.deepcopy(pose.pose)
        new_pose.position.x += offset[0]
        new_pose.position.y += offset[1]
        # rospy.wait_for_service('/gazebo/get_model_state')
        # rospy.wait_for_service('/gazebo/set_model_state')
        set_model_coordinates(ModelState(name, new_pose, pose.twist, 'world'))
    unpause_physics_client()


if __name__ == '__main__':
    spawn_objects()
