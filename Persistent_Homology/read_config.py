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





def main():

    model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    world_properties = rospy.ServiceProxy(
    '/gazebo/get_world_properties', GetWorldProperties)

    world_positions = dict()
    for i in world_properties().model_names:
        if i[0:2] == 'ob':
            position = model_state(i, "world").pose.position
            world_positions[i] = [position.x, position.y]
            print(world_positions[i][0], world_positions[i][1])
    
    return world_positions
if __name__ == '__main__':
    main()
