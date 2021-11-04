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

    # f = open(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'models', 'cylinder', 'model.sdf'))
    # sdff = f.read()
    #
    # obs_f = open(os.path.join(os.path.dirname(os.path.dirname(__file__)),
    #                           'models', 'small_cylinder', 'model.sdf'))
    # obs_sdff = obs_f.read()
    #
    #
    # position_file_address = os.path.join(os.path.dirname(__file__), 'Examples', 's7.txt')

    path_dir = os.path.dirname(os.path.dirname(__file__))

    f = open("/home/ubuntu/Dropbox/Robot/planning_baxter/models/cylinder/model.sdf")

    sdff = f.read()


    obs_f = f = open("/home/ubuntu/Dropbox/Robot/planning_baxter/models/small_cylinder/model.sdf")

    obs_sdff = obs_f.read()


    position_file_address = os.path.join(path_dir, 'Examples', 's1.txt')
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
            x = -0.55 + float(pos[0])
            y = 0.3 + float(pos[1])
            z = 1.08
            spawn_model('small_obstacle_'+str(obs_index), obs_sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            obs_index += 1
        else:
            pos = line.split()
            print("throwing obj %d" % (obj_index))
            x = -0.55 + float(pos[0])
            y = 0.3 + float(pos[1])
            z = 1.08
            spawn_model('object_'+str(obj_index), sdff, "",
                        Pose(Point(x=x, y=y, z=z), orient), "world")
            obj_index += 1
    rospy.signal_shutdown("Fininshed Throwing")
    f.close()
    obs_f.close
    position_file.close()
