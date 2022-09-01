# tip_gripper

import os
import sys
import time
import rospy
import copy
from math import pi, tau
from planit.msg import PercievedObject
from baxter_planit import BaxterPlanner
import moveit_commander
from moveit_commander.conversions import *

def get_arm_position():
    offset_x = 0
    offset_y = 0.56

    rospy.init_node("baxter_planit", anonymous=False)

    planner = BaxterPlanner(False)

    chirality = 'right'


    # the arm can reach
    # x goes from 0.6 to 1.2
    # y goes from 0 to -0.6

    move_group = moveit_commander.MoveGroupCommander(chirality + '_arm')
    # move_group.set_support_surface_name("table__linksurface")

    keep_trying = True
    while keep_trying:    
        wpose = move_group.get_current_pose().pose        
        print(
            "x: " + str(1000*(wpose.position.x)) + " y:" + str(1000*(wpose.position.y-offset_y)) + "\n"
            )
        
        if (wpose.position.y-offset_y < -0.7) or (wpose.position.y-offset_y > 0.1):
            keep_trying = True
            print('wrong arm position')
            continue
        else:
            keep_trying = False

        file_name = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
            'config.txt'
        )
        # print(file_name)
        with open(file_name, 'a') as f:
            f.write('tip_gripper\n')
            f.write(
                str(wpose.position.x) + " " + str(wpose.position.y-offset_y) + " " + str(wpose.position.z) + " " + str(wpose.orientation.x) + " " + str(wpose.orientation.y) + " " + str(wpose.orientation.z) + " " + str(wpose.orientation.w) + "\n"
            )
    

    return wpose.position.x, wpose.position.y-offset_y, wpose.position.z
    

if __name__ == '__main__':
    get_arm_position()