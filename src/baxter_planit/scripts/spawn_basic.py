import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point
import os



if __name__ == '__main__':
    rospy.init_node('spawn_objects')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    orient = Quaternion(0, 0, 0, 1)

    path_dir = os.path.dirname(os.path.dirname(__file__))

    path_models = os.path.join(path_dir, '../../../models')

    f = open(os.path.join(path_models, 'cylinder/model.sdf'))

    sdff = f.read()

    obs_f = open(os.path.join(path_models, 'small_obstacle/model.sdf'))

    obs_sdff = obs_f.read()


    position_file_address = os.path.join(path_dir, 'Examples', 'basic.txt')
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
            print("throwing obstacles %d" % (obs_index))
            x = float(pos[0])
            y = float(pos[1])
            z = 1.08
            print(x, ", " , y, ", ", z)
            spawn_model('small_obstacle_'+str(obs_index), obs_sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            obs_index += 1
        else:
            pos = line.split()
            print("throwing obj %d" % (obj_index))
            x = float(pos[0])
            y = float(pos[1])
            z = 1.08
            print(x, ", " , y, ", ", z)
            spawn_model('object_'+str(obj_index), sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            obj_index += 1
    rospy.signal_shutdown("Fininshed Throwing")
    f.close()
    obs_f.close
    position_file.close()
