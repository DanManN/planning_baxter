#!/usr/bin/env python

import rospy
import tf
import threading
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import *
import time
import os
import random
import rospkg
'''
Should use plugin instead.
'''

if __name__ == '__main__':
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
        elif name[:14] == 'small_obstacle':
            delete_model(name)
            print('deleting', name)
