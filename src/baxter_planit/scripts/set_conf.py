import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, Pose, Point, Twist, Vector3
import os



if __name__ == '__main__':
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    b_x = 0.7  # change of coordinates
    b_y = 0.6  # change of coordinates

    orient = Quaternion(0, 0, 0, 1)

    position_file_address = "config.txt"
    position_file = open(position_file_address, 'r')
    obj_index = 0
    obs_index = 0
    Isobstacle = False
    for line in position_file.readlines():
        print(line)
        if (line == "objects\n"):
            continue
        elif (line == "obstacles\n"):
            Isobstacle = True
            continue
        elif Isobstacle:
            pos = line.split()
            # print("moving obstacles %d" % (obs_index))
            x = float(pos[0]) + b_x
            y = float(pos[1]) + b_y
            z = 1.08

            set_model_state(ModelState('small_obstacle_'+str(obs_index), Pose(Point(x=x, y=y, z=z), orient),
                                       Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0)),
                                       "world"))

            obs_index += 1
        else:
            pos = line.split()
            # print("moving obj %d" % (obj_index))
            x = float(pos[0]) + b_x
            y = float(pos[1]) + b_y
            z = 1.08
            # print(x, ", " , y, ", ", z)

            set_model_state(ModelState('object_'+str(obj_index), Pose(Point(x=x, y=y, z=z), orient),
                                       Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0)),
                                       "world"))

            obj_index += 1

    position_file.close()
