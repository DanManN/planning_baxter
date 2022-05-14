import cv2
import os
from subprocess import Popen, PIPE
import numpy as np
import time
import sys

sys.path.append(
    os.path.dirname(os.path.abspath(__file__))
)
from real.camera import Camera

sys.path.insert(
    0, 
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    )
from constants import cali_tags, target_tag, obstacle_tags

def tag_locations_3d():
    '''
    Input: color img, depth img
    Output: 3d tag location in robot coordinate
    '''
    while 1:
        camera = Camera()
        color_img, depth_img = camera.get_data()
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        cv2.imwrite(os.path.dirname(os.path.abspath(__file__)) + "/real/temp.jpg", color_img)
        p = Popen(
            [os.path.dirname(os.path.abspath(__file__)) + "/real/estimate3d-gui-from-file", os.path.dirname(os.path.abspath(__file__)) + "/real/temp.jpg"],
            stdin=PIPE,
            stdout=PIPE,
            stderr=PIPE,
        )
        
        output, err = p.communicate()
        tag_info = output.decode("utf-8").split("\n")

        tag_info = [t.split() for t in tag_info if len(t.split())==9]
        print(tag_info)
        # tag_dict = {int(t[0]):(float(t[1]), float(t[2])) for t in tag_info}
        tag_arr = np.array([[int(t[0]), (float(t[3])+float(t[5]))/2, (float(t[4])+float(t[6]))/2] for t in tag_info])
        print(tag_arr)
        tag_arr = (np.round_(tag_arr)).astype(int)
        depths = depth_img[tag_arr[:,2], tag_arr[:, 1]]
        for i in range(tag_arr.shape[0]):
            color_img = cv2.circle(color_img, (tag_arr[i][1], tag_arr[i][2]), 4, (255,0,0), 2)
        # cv2.imshow('tag points', color_img)
        # cv2.waitKey(0)
        # cv2.imwrite('ws.jpg', color_img)
        # print(tag_arr)
        # print(depths)
        cam_pts_x = (tag_arr[:, 1]-camera.intrinsics[0][2])*depths[:]/camera.intrinsics[0][0]
        cam_pts_y = (tag_arr[:, 2]-camera.intrinsics[1][2])*depths[:]/camera.intrinsics[1][1]
        cam_pts_z = depths
        cam_pts_x = np.reshape(cam_pts_x,(-1,1))
        cam_pts_y = np.reshape(cam_pts_y,(-1,1))
        cam_pts_z = np.reshape(cam_pts_z,(-1,1))
        # print(cam_pts_x)
        # print(cam_pts_y)
        # print(cam_pts_z)
        cam_pts = np.concatenate((np.reshape(tag_arr[:,0],(-1,1)), cam_pts_x, cam_pts_y, cam_pts_z), axis=1)
        # print(cam_pts)
        cam_pose = np.loadtxt(os.path.dirname(os.path.abspath(__file__)) + "/real/camera_pose.txt", delimiter=" ")
        surface_pts = np.transpose(
            np.dot(cam_pose[0:3, 0:3], np.transpose(cam_pts[:,1:])) + np.tile(cam_pose[0:3, 3:], (1, cam_pts.shape[0]))
        )
        surface_pts = np.concatenate((np.reshape(tag_arr[:,0],(-1,1)), surface_pts), axis=1)
        print(surface_pts)
        print('\n')
        input()

class Perception():

    def __init__(self):
        self.camera = Camera()
            

    
    def update_locations(self):
        # cali_tags = [105, 107, 108, 109]
        # target_tag = 106
        # obstacle_tags = [111, 113,114]
        while 1:
            image, _ = self.camera.get_data()
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            cv2.imwrite(os.path.dirname(os.path.abspath(__file__)) + "/real/temp.jpg", image)
            p = Popen(
                [os.path.dirname(os.path.abspath(__file__)) + "/real/estimate3d-gui-from-file", os.path.dirname(os.path.abspath(__file__)) + "/real/temp.jpg"],
                stdin=PIPE,
                stdout=PIPE,
                stderr=PIPE,
            )
            # p = Popen(
            #     [os.path.dirname(__file__) + "/real/detect-from-file", os.path.dirname(__file__) + "/real/temp.jpg"],
            #     stdin=PIPE,
            #     stdout=PIPE,
            #     stderr=PIPE,
            # )
            output, err = p.communicate()
            tag_info = output.decode("utf-8").split("\n")

            tag_info = [t.split() for t in tag_info if len(t.split())==9]
            tag_info = {int(t[0]):(float(t[1]), float(t[2])) for t in tag_info}
            # print("tag_info", tag_info)
            robot_coor = {k:self.camera.camera_to_robot(tag_info[k]) for k in tag_info.keys()}
            l = sorted(list(robot_coor.items()), key=lambda e:e[0])
            print(*l, sep = '\n')
            print('\n')
            time.sleep(0.1)
            print('##################')
            if set(list(robot_coor)) == set(cali_tags+obstacle_tags+[target_tag]):
                print('Successful perception')
                print('##################')
                break
            else:
                print(' Failed perception, will try again.')
                # time.sleep(0.3)
            print('##################')
            



        config_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'config.txt')
        with open(config_file, 'w') as f:
            f.write('object\n')
            f.write(str(robot_coor[target_tag][0])+' '+str(robot_coor[target_tag][1])+'\n')
            f.write('obstacle\n')
            for k in robot_coor.keys():
                if (k in cali_tags) or (k==target_tag):
                    continue
                else:
                    f.write(str(robot_coor[k][0])+' '+str(robot_coor[k][1])+'\n')

    # if __name__ == '__main__':
    #     # perception(need_cali)
    #     perception()