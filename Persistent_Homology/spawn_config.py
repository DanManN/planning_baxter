import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point
import os
import copy

from std_srvs.srv import Empty
pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')

get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_model_coordinates = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


def get_workspace_center():
    pose = model_coordinates('boundaryS', 'world')
    x, z = pose.pose.position.x, pose.pose.position.z
    pose = model_coordinates('boundaryW', 'world')
    y = pose.pose.position.y
    print("x,y,z", x, y, z)
    return x, y, z

def delete_model():
    rospy.init_node('delete_objects')
    rospy.wait_for_service('/gazebo/delete_model')
    rospy.wait_for_service('/gazebo/get_world_properties')

    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

    properties = get_world_properties()
    for name in properties.model_names:
        if name[:6] == 'object':
            delete_model(name)
            print('deleting', name)
        elif name[:4] == 'obst':
            delete_model(name)
            print('deleting', name)
        elif name[:5] == 'stick':
            delete_model(name)
            print('deleting', name)


def spawn_config(position_file_address, all_items):
    # rospy.init_node('spawn_objects')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    orient = Quaternion(0, 0.7071068, 0, 0.7071068)

    path_dir = os.path.dirname(__file__)

    path_models = os.path.join(path_dir, '../models')


    f_all_items = dict()

    for k, name_obstacle in enumerate(all_items):

        f_all_items[k]=open(os.path.join(path_models, f'{name_obstacle}/model.sdf'))

    stick_f = open(os.path.join(path_models, 'stick/model.sdf'))

    ws_x, ws_y, ws_z = get_workspace_center()
    ws_z = 1.077  # for obstacles and objects
    y_shift = 0
    flag = False

    positions_file = open(position_file_address, 'r')
    for line in positions_file.readlines():
        print(line)
        if (line == "object\n"):
            name = line[0:-1]
            index = 0

        elif (line == "obstacle\n"):
            name = line[0:-1]

        elif (line == "tip_gripper\n"):
            # name = line[0:-1]
            orient = Quaternion(0, -0.7071067811865475, 0, 0.7071067811865475)
            name = "stick"
            temp_sdff = stick_f.read()

        else:
            pos = line.split()
            x = float(pos[0])
            y = float(pos[1]) + y_shift
            z = 1.077

            if name != 'stick':
                if index < len(all_items):
                    # temp_sdff = f_all_items[1].read()
                # else:
                    temp_sdff = f_all_items[index].read()
                temp_name = name+"_"+str(index)
            else:
                lengh_gripper2elbow = 0.3575
                x= x - lengh_gripper2elbow
                temp_name = name
                z = ws_z
                flag = True

            spawn_model(temp_name, temp_sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            index += 1

    for k, name_obstacle in enumerate(all_items):
        f_all_items[k].close()

    if flag:
        stick_f.close()
    positions_file.close()


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


def main():

    all_items = ['cylinder_object',
    '042_can', 
    '043_cracker_box', 
    # '004_sugar_box_textured', 
    '045_soup_can', 
    '041_mustard_bottle_textured',  
    '048_pudding_box',
    '049_gelatin_box',
    '050_potted_meat_can',
    # '021_bleach_cleanser_textured',
    'cylinder',
    ]# include objects and obstacles

    # all_items = ['cylinder_object',
    # 'cylinder',
    # ]# include objects and obstacles

    position_file_address = os.path.join(
        os.path.dirname(os.path.dirname((os.path.abspath(__file__)))),
        "config.txt"
    )

    # position_file_address = os.path.join(
    #     os.path.dirname(os.path.dirname((os.path.abspath(__file__)))), "Persistent_Homology/examples/"
    #     "config_lenghty.txt"
    # )
    # name_file="config.txt"

    delete_model()
    spawn_config(position_file_address, all_items)


if __name__ == '__main__':
    main()
