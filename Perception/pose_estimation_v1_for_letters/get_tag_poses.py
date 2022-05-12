import numpy as np
from subprocess import Popen, PIPE
import cv2
import math
from matplotlib import pyplot as plt
import os
import time

from real.camera import Camera
import utils
from constants import workspace_limits
from get_poses import Perception

heightmap_resolution = 0.001
background_threshold = {"low": np.array([0, 0, 0], np.uint8), "high": np.array([255, 255, 255], np.uint8)}

class Tag_Perception(Perception):
    def __init__(self, pool = 'a'):
        self.camera = Camera()
        self.cam_pose = np.loadtxt(os.path.dirname(os.path.abspath(__file__)) + "/real/camera_pose.txt", delimiter=" ")
        # Set the letters should appear in the scene
        pool = list(pool)
        self.pool = {}
        for let in pool:
            if let in self.pool:
                self.pool[let] += 1
            else:
                self.pool[let] = 1
    
    ######################################## process image ########################################
    def get_camera_data(self):
        # Get color and depth image from ROS service
        color_img, depth_img = self.camera.get_data()
        # Remove background
        # img = cv2.cvtColor(color_img, cv2.COLOR_RGB2HSV)
        # bg_mask = cv2.inRange(img, background_threshold["low"], background_threshold["high"])
        # color_img = cv2.bitwise_and(color_img, color_img, mask=bg_mask)
        return color_img, depth_img


    ######################################## get tag arrangement ########################################
    def get_arrangement(self, purpose=None):
        arr_list = self.get_poses()
        print(arr_list)
        if purpose == "G":
            with open("/home/gemc/real_general_object_experiments/demo/tags/tags_Goal.txt", "w") as f:
                f.write("objects\n")
                ordered = sorted(arr_list, key=lambda x: x[3])
                for line in ordered:
                    txt = str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + " " + "a" + "\n"
                    f.write(txt)
        elif purpose == "S":
            with open("/home/gemc/real_general_object_experiments/demo/tags/tags_Start.txt", "w") as f:
                f.write("objects\n")
                ordered = sorted(arr_list, key=lambda x: x[3])
                for line in ordered:
                    txt = str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + " " + "a" + "\n"
                    f.write(txt)

    def get_poses(self):
        center = (0.0, -0.5)
        poses = []
        print('pool', self.pool)
        while len(poses) < sum(list(self.pool.values())):
            poses = []
            # Get latest RGB-D image
            color_img, _ = self.get_camera_data()
            tag_dict = self.check_tag_pixel_points(color_img)
            print('try detect tags')
            for k, v in tag_dict.items():
                if k in [109, 113, 110, 129]:
                    continue
                p1, p2, p3 = v
                # p1 = np.array(p1, dtype=np.float32)
                # p2 = np.array(p2, dtype=np.float32)
                # p3 = np.array(p3, dtype=np.float32)
                # p1 = np.round_(p1)
                # p2 = np.round_(p2)
                # p3 = np.round_(p3)
                # color_img = cv2.circle(color_img, tuple(p1), 3, (255,0,0), thickness = -1)
                # color_img = cv2.circle(color_img, tuple(p2), 3, (255,0,0), thickness = -1)
                # color_img = cv2.circle(color_img, tuple(p3), 3, (255,0,0), thickness = -1)
                # color_img = cv2.circle(color_img, (int((p2[0]+p3[0])/2.0), int((p2[1]+p3[1])/2.0)), 3, (255, 0, 0), thickness = -1)
                p1 = utils.camera_to_robot(np.array(p1, dtype=np.float32), self.camera.camera_to_robot_matrix)
                p2 = utils.camera_to_robot(np.array(p2, dtype=np.float32), self.camera.camera_to_robot_matrix)
                p3 = utils.camera_to_robot(np.array(p3, dtype=np.float32), self.camera.camera_to_robot_matrix)
                tag_center = ((p2[0]+p3[0])/2.0, (p2[1]+p3[1])/2.0)
                theta = math.atan((p2[1]-p1[1])/(p2[0]-p1[0]))
                theta = utils.back2bound(theta, -math.pi, math.pi)
                poses.append((100*(tag_center[0]-center[0]), 100*(tag_center[1]-center[1]), theta, k))
                # print(k, p1, theta)
            # print('detected_pose :', poses)
            # print(self.pool)
        # plt.imshow(color_img)
        # plt.show()
        return poses
        
    def check_tag_pixel_points(self, color_img):
        cv2.imwrite(os.path.dirname(__file__) + "/real/temp.jpg", color_img)
        p = Popen(
            [os.path.dirname(os.path.abspath(__file__)) + "/real/estimate3d-gui-from-file", os.path.dirname(os.path.abspath(__file__)) + "/real/temp.jpg"],
            stdin=PIPE,
            stdout=PIPE,
            stderr=PIPE,
        )
        output, err = p.communicate()
        tag_info = output.decode("utf-8").split("\n")

        tag_info = [t.split() for t in tag_info if len(t.split())==9]
        print(*tag_info, sep='\n')
        print('\n')
        tag_info = {int(t[0]):[(t[1], t[2]), (t[3], t[4]), (t[5], t[6])] for t in tag_info}
        
        return tag_info       

    def update_pose(self, curr_pose):
        arr_list = self.get_poses()
        ordered = sorted(arr_list, key=lambda x: x[3])
        shortest_dist = float('inf')
        closest_pose = None
        for pose in ordered:
            dist = np.linalg.norm(
                np.array(curr_pose[:2])-np.array(pose[:2])
            )
            if dist <= shortest_dist:
                closest_pose = pose
                shortest_dist = dist
        return closest_pose


if __name__ == '__main__':
    numObjs = 1
    pool = ''
    for _ in range(numObjs):
        pool += 'a'
    P = Tag_Perception(pool=pool)
    # print(P.get_arrangement(purpose='G'))