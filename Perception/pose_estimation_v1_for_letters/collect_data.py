from numpy.core.fromnumeric import transpose
from real.camera import Camera
import utils
from constants import workspace_limits
import cv2
import numpy as np
import math
import copy
import matplotlib.pyplot as plt
import os


heightmap_resolution = 0.001
background_threshold = {"low": np.array([0, 0, 125], np.uint8), "high": np.array([255, 255, 255], np.uint8)}


class Perception(object):
    def __init__(self, pool="ICRA2022"):
        self.camera = Camera()
        self.cam_pose = np.loadtxt("real/camera_pose.txt", delimiter=" ")
        pool = list(pool)
        self.pool = {}
        for let in pool:
            if let in self.pool:
                self.pool[let] += 1
            else:
                self.pool[let] = 1
        self.load_models()

    def load_models(self):
        dir_path = os.path.join(os.path.dirname(__file__), "models")
        self.des = {}
        self.kp = {}
        # for file_name in os.listdir(dir_path):
        for file_name in self.pool.keys():
            # print("letter", file_name)
            img = cv2.imread(os.path.join(dir_path, file_name, "mask.png"))
            # img = cv2.imread(os.path.join(dir_path, file_name, "heightmap_mask.png"))
            # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # _, img = cv2.threshold(img, 10, 255, cv2.THRESH_BINARY)
            # print("gray shape", img.shape)
            # print("type", type(img))
            # Initiate ORB detector
            orb = cv2.ORB_create()
            # find the keypoints and descriptors with orb
            self.kp[file_name], self.des[file_name] = orb.detectAndCompute(img, None)
            # print("load", self.des[file_name])
            # cv2.imshow("frame", img)
            # cv2.waitKey(0)

    def get_arrangement(self):
        # Get latest RGB-D image
        color_img, depth_img = self.get_camera_data()
        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        color_heightmap, depth_heightmap, real_depth_heightmap = utils.get_heightmap(
            color_img, depth_img, self.camera.intrinsics, self.cam_pose, workspace_limits, heightmap_resolution
        )
        # cv2.imshow("frame", color_heightmap)
        # cv2.waitKey(0)
        cv2.imwrite("start.png", color_heightmap)
        arr_list = self.get_contours(color_heightmap, depth_heightmap, real_depth_heightmap, color_img, depth_img)
        return arr_list

    def feature_matching(self, img):
        print("mask shape:", img.shape)
        cv2.imwrite("mask.png", img)
        # dir_path = os.path.join(os.path.dirname(__file__), "models")
        # Initiate ORB detector
        orb = cv2.ORB_create()
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # _, img = cv2.threshold(img, 10, 255, cv2.THRESH_BINARY)
        # print("gray shape", img.shape)
        # print("type", type(img))
        # find the keypoints and descriptors with orb
        kp, des = orb.detectAndCompute(img, None)

        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        # bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=False)
        score_dict = {}
        for letter, letter_des in self.des.items():
            # print("des", des),
            # print("letter des", letter_des)
            # Match descriptors.
            matches = bf.match(des, letter_des)
            # Sort them in the order of their distance.
            matches = sorted(matches, key=lambda x: x.distance)
            score_dict[letter] = [m.distance for m in matches]
            # score = np.average([m.distance for m in matches[:5]])
            # if best_score > score:
            #     best_score = score
            #     best_match = letter
            print(letter)
            print("total:", len(matches))
            print("top distance:", np.average([m.distance for m in matches[:5]]))
        min_features = np.min([len(v) for v in score_dict.values()])
        for letter in score_dict.keys():
            score_dict[letter] = np.average(score_dict[letter][:min_features])
        # cv2.imshow("frame", img)
        # cv2.waitKey(0)
        sorted_letter_list = sorted(score_dict.keys(), key=lambda x: score_dict[x])
        print("sorted_list", sorted_letter_list)
        print("confidence", score_dict[sorted_letter_list[1]] / score_dict[sorted_letter_list[0]])
        # cv2.imshow("frame", img)
        # cv2.waitKey(0)
        return sorted_letter_list, score_dict[sorted_letter_list[1]] / score_dict[sorted_letter_list[0]]

    def get_real_point_cloud(self, x, y, w, h, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, img = cv2.threshold(img, 10, 255, cv2.THRESH_BINARY)
        rpc = []
        for i in range(x, x + w + 1):
            for j in range(y, y + h + 1):
                if img[j, i] >= 100:
                    rpc.append(
                        [
                            i * heightmap_resolution + workspace_limits[0][0],
                            j * heightmap_resolution + workspace_limits[1][0],
                        ]
                    )
                    # print(
                    #     [
                    #         j * heightmap_resolution + workspace_limits[0][0],
                    #         i * heightmap_resolution + workspace_limits[1][0],
                    #     ]
                    # )
        # for p in rpc:
        #     plt.scatter(p[0], p[1])
        # plt.show()
        return rpc

    def get_camera_data(self):
        # Get color and depth image from ROS service
        color_img, depth_img = self.camera.get_data()
        # Remove background
        img = cv2.cvtColor(color_img, cv2.COLOR_RGB2HSV)
        bg_mask = cv2.inRange(img, background_threshold["low"], background_threshold["high"])
        color_img = cv2.bitwise_and(color_img, color_img, mask=bg_mask)

        return color_img, depth_img

    def get_contours(self, color_heightmap, depth_heightmap, real_depth_heightmap, color_img, depth_img):
        gray = cv2.cvtColor(color_heightmap, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
        contours = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        arr_list = []
        masks2models = {}
        mask_confidence = {}
        mask_bb = {}
        for i, cnt in enumerate(contours):
            x, y, w, h = cv2.boundingRect(cnt)
            if w <= 20 or h <= 20:
                continue
            roi = color_heightmap[y : y + h, x : x + w]
            # roi = cv2.copyMakeBorder(roi, 20, 20, 20, 20, cv2.BORDER_CONSTANT, None, (0, 0, 0))
            pts = np.array(
                (
                    (x, y),
                    (x + w, y),
                    (x + w, y + h),
                    (x, y + h),
                )
            )
            pix_pts = utils.reverse_heightmap(
                pts,
                self.camera.intrinsics,
                self.cam_pose,
                workspace_limits,
                heightmap_resolution,
                real_depth_heightmap,
                depth_img,
                color_heightmap,
            )
            roi = color_img[pix_pts[0][1] : pix_pts[1][1], pix_pts[0][0] : pix_pts[1][0]]
            # cv2.rectangle(color_img, pix_pts[0], pix_pts[1], (200, 0, 0), 2)
            roi = cv2.copyMakeBorder(roi, 20, 20, 20, 20, cv2.BORDER_CONSTANT, None, (0, 0, 0))
            # cv2.imshow("frame", roi)
            # cv2.waitKey(0)
            masks2models[i], mask_confidence[i] = self.feature_matching(roi)
            mask_bb[i] = (x, y, w, h)
        # cv2.imshow("frame", color_img)
        # cv2.waitKey(0)
        # cv2.imwrite("bb.png", color_img)
        sorted_masks = sorted(mask_confidence.keys(), key=lambda x: mask_confidence[x], reverse=True)
        print("sorted_masks", sorted_masks)
        for maskID in sorted_masks:
            let = None
            for letter in masks2models[maskID]:
                if letter in self.pool:
                    if self.pool[letter] > 0:
                        let = letter
                        self.pool[letter] -= 1
                        break
                    else:
                        continue
                else:
                    print("Mismatch!")
                    continue
            if let == None:
                break
            x, y, w, h = mask_bb[maskID]
            # mask_pc = self.get_real_point_cloud(x, y, w, h, color_heightmap)
            # pose = self.get_transformation(np.array(mask_pc), let)
            # print("predicted pose:", pose + [let])
            roi = color_heightmap[y : y + h, x : x + w]
            roi = cv2.copyMakeBorder(roi, 20, 20, 20, 20, cv2.BORDER_CONSTANT, None, (0, 0, 0))
            self.save_labeled_img(roi, let)
            # arr_list.append(tuple(pose + [let]))
        if let != None:
            return arr_list

    def save_labeled_img(self, img, label):
        dir_path = os.path.join(os.path.dirname(__file__), 'letter_dataset', label)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

        idx = len(os.listdir(dir_path))
        cv2.imwrite(
            os.path.join(dir_path, str(idx)+'.png'), img)


    def reverse_compute(self):
        # Get latest RGB-D image
        color_img, depth_img = self.get_camera_data()
        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        color_heightmap, depth_heightmap = utils.get_heightmap_copy(
            color_img, depth_img, self.camera.intrinsics, self.cam_pose, workspace_limits, heightmap_resolution
        )
        cv2.imshow("frame", color_heightmap)
        cv2.waitKey(0)
        cv2.imwrite("start.png", color_heightmap)

    def check_pose_correctness(self, pose, letter, mask_pc):
        center = np.array([0, -0.5])
        ori_pc = self.get_model_pc(letter)
        R = np.array([[math.cos(pose[2]), -math.sin(pose[2])], [math.sin(pose[2]), math.cos(pose[2])]])
        ori_pc[:, 0] -= center[0]
        ori_pc[:, 1] -= center[1]
        trans_pc = np.transpose(np.dot(R, np.transpose(ori_pc)))
        trans_pc[:, 0] += pose[0] + center[0]
        trans_pc[:, 1] += pose[1] + center[1]

        for p in trans_pc:
            plt.scatter(p[0], p[1])
        for p in mask_pc:
            plt.scatter(p[0], p[1])
        plt.show()

    def get_transformation(self, mask_pc, letter):
        ori_pc = self.get_model_pc(letter)
        ori_center = np.array([np.average(ori_pc[:, 0]), np.average(ori_pc[:, 1])])
        mask_center = np.array([np.average(mask_pc[:, 0]), np.average(mask_pc[:, 1])])

        translation = mask_center - ori_center

        # print("mask_center", mask_center)
        # print("ori_center", ori_center)
        # print("trans", translation)

        shifted_ori = ori_pc + translation
        best_angle = 0
        best_score = 0
        # for ang in range(0, 360):
        for ang in np.arange(0, 360, 1):
            score = self.evaluate_rotation(shifted_ori, mask_pc, mask_center, ang * np.pi / 180)
            if score > best_score:
                best_score = score
                best_angle = ang * np.pi / 180

        correct_translation = self.get_translation(letter, ori_center, translation, best_angle)
        return [correct_translation[0], correct_translation[1], best_angle]

    def get_translation(self, shape, ori_center, translation, angle):
        # g = np.array(self.get_grasping_pose(shape))
        g = np.array((0, -0.5))
        c = np.array(ori_center)
        R_I = np.array([[math.cos(angle) - 1, -math.sin(angle)], [math.sin(angle), math.cos(angle) - 1]])
        t = np.array(translation)
        # print("R_I", R_I)
        # print("diff", g - c)

        return np.dot(R_I, g - c) + t

    def evaluate_rotation(self, src_pc, tgt_pc, center, angle):
        scale = 800
        rotated_pc = np.zeros(src_pc.shape)
        rotated_pc[:, 0] = np.round_(
            scale
            * (math.cos(angle) * (src_pc[:, 0] - center[0]) - math.sin(angle) * (src_pc[:, 1] - center[1]) + center[0])
        )
        rotated_pc[:, 1] = np.round_(
            scale
            * (math.sin(angle) * (src_pc[:, 0] - center[0]) + math.cos(angle) * (src_pc[:, 1] - center[1]) + center[1])
        )

        src_minx = min(rotated_pc[:, 0])
        src_maxx = max(rotated_pc[:, 0])
        src_miny = min(rotated_pc[:, 1])
        src_maxy = max(rotated_pc[:, 1])

        # print(src_miny, src_maxy)
        # print(rotated_pc)
        tgt_pc_copy = np.zeros(tgt_pc.shape)
        # tgt_pc_copy[:,0] = np.round_(scale*(math.cos(angle)*(tgt_pc[:,0]-center[0])-math.sin(angle)*(tgt_pc[:,1]-center[1])+center[0]))
        # tgt_pc_copy[:,1] = np.round_(scale*(math.sin(angle)*(tgt_pc[:,0]-center[0])+math.cos(angle)*(tgt_pc[:,1]-center[1])+center[1]))
        tgt_pc_copy[:, 0] = np.round_(scale * tgt_pc[:, 0])
        tgt_pc_copy[:, 1] = np.round_(scale * tgt_pc[:, 1])

        # print(tgt_pc_copy)

        tgt_minx = min(tgt_pc_copy[:, 0])
        tgt_maxx = max(tgt_pc_copy[:, 0])
        tgt_miny = min(tgt_pc_copy[:, 1])
        tgt_maxy = max(tgt_pc_copy[:, 1])

        bb_minx = min(src_minx, tgt_minx)
        bb_maxx = max(src_maxx, tgt_maxx)
        bb_miny = min(src_miny, tgt_miny)
        bb_maxy = max(src_maxy, tgt_maxy)

        # print(bb_minx, bb_miny, bb_maxx, bb_maxy)

        src_img = np.zeros((int(bb_maxx - bb_minx + 1), int(bb_maxy - bb_miny + 1)))
        tgt_img = np.zeros((int(bb_maxx - bb_minx + 1), int(bb_maxy - bb_miny + 1)))
        for p in rotated_pc:
            src_img[int(p[0] - bb_minx), int(p[1] - bb_miny)] = 255

        for p in tgt_pc_copy:
            tgt_img[int(p[0] - bb_minx), int(p[1] - bb_miny)] = 255

        and_img = cv2.bitwise_and(src_img, tgt_img)
        or_img = cv2.bitwise_or(src_img, tgt_img)
        int_area = 0
        union_area = 0
        for i in and_img:
            for val in i:
                if val != 0:
                    int_area += 1
            # print(int_area)
        # int_area = sum(and_img!=0)
        for i in or_img:
            for val in i:
                if val != 0:
                    union_area += 1
        # if (int_area/union_area > 0.85):
        # cv2.imshow('frame',src_img)
        # cv2.imwrite("models/R/R" + str(angle) + ".png", src_img)
        # cv2.waitKey(0)
        # cv2.imshow('frame',tgt_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # print("angle:" + str(angle))
        # print(int_area)
        # print(union_area)
        # print(int_area/union_area)
        return int_area / union_area

    def get_model_pc(self, letter):
        dir_path = os.path.join(os.path.dirname(__file__), "models")
        return np.loadtxt(os.path.join(dir_path, letter, "pc.txt"))


if __name__ == "__main__":
    P = Perception()
    P.get_arrangement()
