import os
import time
import numpy as np
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'baxter_planit', 'scripts'))
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point

from Perception.pose_estimation_v1_for_letters.get_tags import Perception
# from src.baxter_planit.scripts.position_the_arm import position_the_arm as sim_position_the_arm
from src.real_baxter_planit.scripts.position_the_arm import position_the_arm as real_position_the_arm
from src.real_baxter_planit.scripts.get_arm_position import get_arm_position as real_get_arm_position
from src.real_baxter_planit.scripts.demo_read_plan import demo_real_plan as real_execute
from src.baxter_planit.scripts.PHIA_1st import PHIA_pipeline as sim_get_plan


def sim_spawn_objects():
    '''
    Input: config.txt (new object states)
    Output: objects in gazebo
    '''
    # delete current objects
    sim_delete_models()

    # position the arm
    sim_position_the_arm()

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
        if (line == "objects\n"):
            continue
        elif (line == "obstacles\n"):
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
        elif name[:14] == 'small_obstacle':
            delete_model(name)
            print('deleting', name)


def real_perception(P:Perception):
    # cmd = 'python ./Perception/pose_estimation_v1_for_letters/get_tags.py'
    P.update_locations()

def real_calibration():
    return Perception()

def check_success():
    plan_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'plan.txt'
    )
    with open(plan_file, 'r') as f:
        if len(list(f.readlines())) <4:
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

def pipeline():
    time_perception = 0
    time_get_arm_current_position = 0
    time_task_planning = 0
    time_success = 0
    time_motion_planning = 0
    start = time.time()
    real_position_the_arm()
    print('real_withdraw time' , time.time()-start)
    input('Enter')
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
        sim_get_plan()
        time_task_planning = time.time()-start
        print('perception: ', time_perception)
        print('check arm: ', time_get_arm_current_position)
        print('task planning ', time_task_planning)
        # input('Plan ready')
        start = time.time()
        if check_success():
            print('perception: ', time_perception)
            print('check arm: ', time_get_arm_current_position)
            print('task planning ', time_task_planning)
            break
        print('simulation plan and execute time' , time.time()-start)
        real_execute()
        time_motion_planning = time.time()-start
        print('finish executing')
        print('perception: ', time_perception)
        print('check arm: ', time_get_arm_current_position)
        print('task planning ', time_task_planning)
        print('motion planning and execution: ', time_motion_planning)

        input('Enter')
        

if __name__ == '__main__':
    pipeline()
    # real_perception()
    # real_get_arm_position()
    # real_execute()
    # sim_get_plan()
    # test()