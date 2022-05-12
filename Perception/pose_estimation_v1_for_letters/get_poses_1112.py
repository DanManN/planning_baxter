import sys, os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pose_estimation_v1_for_letters.ICP import ICP

sys.path.append(os.path.dirname(__file__))
from real.camera import Camera
import utils
from constants import workspace_limits
import cv2
import numpy as np
import math
import copy
import matplotlib.pyplot as plt

import pre_trained_model.get_mask_from_pre_trained_model as model
import time
from subprocess import Popen, PIPE

letter_map = {0: "B", 1: "C", 2: "E", 3: "G", 4: "I", 5: "0", 6: "R", 7: "S", 8: "T", 9: "U"}

heightmap_resolution = 0.001
background_threshold = {"low": np.array([0, 0, 140], np.uint8), "high": np.array([255, 255, 255], np.uint8)}
# I_threshold = {"low": np.array([101, 0, 0], np.uint8), "high": np.array([255, 255, 255], np.uint8)}
# I_threshold = {"low": np.array([0, 0, 165], np.uint8), "high": np.array([16, 83, 255], np.uint8)}
# A_threshold = {"low": np.array([103, 240, 104], np.uint8), "high": np.array([255, 255, 255], np.uint8)}


class Perception(object):
    def __init__(self, pool="RUTGERSR0B0TICS"):
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
        # load pre-trained model
        self.predictor = model.load_model()


    def save_pose(self, text="R"):
        # Get latest RGB-D image
        color_img, depth_img = self.get_camera_data()
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        color_heightmap, depth_heightmap, real_depth_heightmap = utils.get_both_heightmaps(
            color_img,
            depth_img,
            self.camera.intrinsics,
            self.cam_pose,
            workspace_limits,
            heightmap_resolution,
            self.camera.camera_to_robot_matrix,
            self.camera.robot_to_camera_matrix,
        )
        cv2.imshow("frame", color_img)
        cv2.waitKey(0)
        cv2.imwrite("start.png", color_img)
        self.save_contour(text, color_heightmap, depth_heightmap, real_depth_heightmap, color_img, depth_img)

    def save_contour(self, text, color_heightmap, depth_heightmap, real_depth_heightmap, color_img, depth_img):
        my_path = os.path.join(os.path.dirname(__file__), "models", text)
        if not os.path.exists(my_path):
            os.makedirs(my_path)
        gray = cv2.cvtColor(color_heightmap, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("frame", gray)
        # cv2.waitKey(0)
        _, binary = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
        # cv2.imshow("frame", binary)
        # cv2.waitKey(0)
        contours = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        idx = 0
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w <= 20 or h <= 20:
                continue
            idx += 1
            roi = color_heightmap[y : y + h, x : x + w]
            roi = cv2.copyMakeBorder(roi, 20, 20, 20, 20, cv2.BORDER_CONSTANT, None, (0, 0, 0))
            cv2.imwrite(os.path.join(my_path, "heightmap_mask.png"), roi)
            cv2.imwrite(os.path.join(my_path, "heightmap.png"), color_heightmap)
            cv2.imshow("frame", roi)
            cv2.waitKey(0)
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
            roi = cv2.copyMakeBorder(roi, 20, 20, 20, 20, cv2.BORDER_CONSTANT, None, (0, 0, 0))
            cv2.imwrite(os.path.join(my_path, "mask.png"), roi)
            cv2.imwrite(os.path.join(my_path, "original.png"), color_img)
            rpc = self.get_real_point_cloud(x, y, w, h, color_heightmap)
            np.savetxt(os.path.join(my_path, "pc.txt"), np.array(rpc))

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

    def get_arrangement(self, purpose=None):
        '''
        Detect objects and get poses
        '''
        # Get latest RGB-D image
        color_img, depth_img = self.get_camera_data()
        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        color_heightmap = utils.get_heightmap(
            color_img,
            depth_img,
            self.camera.intrinsics,
            self.cam_pose,
            workspace_limits,
            heightmap_resolution,
            self.camera.camera_to_robot_matrix,
            self.camera.robot_to_camera_matrix,
        )
        if purpose == "G":
            arr_list = self.get_poses(color_heightmap)
            with open("/home/gemc/real_general_object_experiments/demo/letters/letters_Goal.txt", "w") as f:
                f.write("objects\n")
                ordered = sorted(arr_list, key=lambda x: x[3])
                for line in ordered:
                    txt = str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + " " + str(line[3]) + "\n"
                    f.write(txt)
        elif purpose == "S":
            arr_list = self.get_poses(color_heightmap)
            with open("/home/gemc/real_general_object_experiments/demo/letters/letters_Start.txt", "w") as f:
                f.write("objects\n")
                ordered = sorted(arr_list, key=lambda x: x[3])
                for line in ordered:
                    txt = str(line[0]) + " " + str(line[1]) + " " + str(line[2]) + " " + str(line[3]) + "\n"
                    f.write(txt)
        return arr_list

    def get_real_2d_point_cloud_from_binary_mask(self, mask):
        # True or False
        indices = np.where(mask == True)
        num_points = len(indices[0])
        indices = np.array(indices)
        rpc = np.zeros([num_points, 2])
        rpc[:,0] = indices[0,:] * heightmap_resolution + workspace_limits[0][0]
        rpc[:,1] = indices[1,:] * heightmap_resolution + workspace_limits[1][0]
        return rpc

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

    def get_real_3d_point_cloud_from_binary_mask(self, mask, depth_heightmap):
        # True or False
        indices = np.where(mask == True)
        num_points = len(indices[0])
        rpc = []
        for p in range(num_points):
            j = indices[0][p]
            i = indices[1][p]
            rpc.append(
                [
                    i * heightmap_resolution + workspace_limits[0][0],
                    j * heightmap_resolution + workspace_limits[1][0],
                    depth_heightmap[j, i],
                ]
            )
        return rpc

    def get_camera_data(self):
        # Get color and depth image from ROS service
        color_img, depth_img = self.camera.get_data()
        # cv2.imshow("frame", color_img)
        # cv2.waitKey(0)
        # Remove background
        img = cv2.cvtColor(color_img, cv2.COLOR_RGB2HSV)
        # cv2.imshow("frame", img)
        # cv2.waitKey(0)
        # A_mask = cv2.inRange(img, A_threshold["low"], A_threshold["high"])
        # cv2.imshow("A", A_mask)
        # cv2.waitKey(0)
        bg_mask = cv2.inRange(img, background_threshold["low"], background_threshold["high"])
        # cv2.imshow("bg", bg_mask)
        # cv2.waitKey(0)
        # bg_mask = cv2.add(bg_mask, A_mask)
        # cv2.imshow("bg", bg_mask)
        # cv2.waitKey(0)
        color_img = cv2.bitwise_and(color_img, color_img, mask=bg_mask)
        # cv2.imshow("frame", color_img)
        # cv2.waitKey(0)
        return color_img, depth_img

    def get_poses(self, color_heightmap):
        '''
        Get object poses in the scene
        '''
        outputs = self.predictor(color_heightmap)
        while not self.segmentation_correct(outputs):
            print("wrong segmentation, try again")
            # Get latest RGB-D image
            color_img, depth_img = self.get_camera_data()
            # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
            color_heightmap = utils.get_heightmap(
                color_img,
                depth_img,
                self.camera.intrinsics,
                self.cam_pose,
                workspace_limits,
                heightmap_resolution,
                self.camera.camera_to_robot_matrix,
                self.camera.robot_to_camera_matrix,
            )
            self.predictor = model.update_score_threshold(-0.05)
            outputs = self.predictor(color_heightmap)
        self.predictor = model.restore_score_threshold(0.7)
        num_instances = outputs["instances"].pred_boxes.tensor.numpy().shape[0]
        arr_list = []
        for obj in range(num_instances):
            let = letter_map[int(outputs["instances"].pred_classes[obj])]
            # self.get_accurate_mask(outputs, obj, color_heightmap)
            mask_pc = self.get_real_2d_point_cloud_from_binary_mask(outputs["instances"].pred_masks[obj])
            pose = self.get_transformation(np.array(mask_pc), let)
            arr_list.append(tuple(pose + [let]))
        # m to cm
        arr_list = [(100.0 * pose[0], 100.0 * pose[1], pose[2], pose[3]) for pose in arr_list]
        return arr_list

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
        for p in ori_pc:
            plt.scatter(p[0], p[1], facecolor="y")
        R = np.array([[math.cos(pose[2]), -math.sin(pose[2])], [math.sin(pose[2]), math.cos(pose[2])]])
        ori_pc[:, 0] -= center[0]
        ori_pc[:, 1] -= center[1]
        trans_pc = np.transpose(np.dot(R, np.transpose(ori_pc)))
        trans_pc[:, 0] += pose[0] + center[0]
        trans_pc[:, 1] += pose[1] + center[1]

        for p in trans_pc:
            plt.scatter(p[0], p[1], facecolor="r")
        for p in mask_pc:
            plt.scatter(p[0], p[1], facecolor="b")
        plt.show()

    def get_transformation(self, mask_pc, letter):
        ori_pc = self.get_model_pc(letter)
        ori_center = np.array([np.average(ori_pc[:, 0]), np.average(ori_pc[:, 1])])
        mask_center = np.array([np.average(mask_pc[:, 0]), np.average(mask_pc[:, 1])])

        translation = mask_center - ori_center

        shifted_pc = ori_pc + translation

        scale = 800
        shifted_pc = np.round_(scale * shifted_pc)
        mask_pc = np.round_(scale * mask_pc)
        rotation_center = scale * mask_center

        src_minx = min(shifted_pc[:, 0])
        src_maxx = max(shifted_pc[:, 0])
        src_miny = min(shifted_pc[:, 1])
        src_maxy = max(shifted_pc[:, 1])

        tgt_minx = min(mask_pc[:, 0])
        tgt_maxx = max(mask_pc[:, 0])
        tgt_miny = min(mask_pc[:, 1])
        tgt_maxy = max(mask_pc[:, 1])

        bb_minx = min(src_minx, tgt_minx) - 5
        bb_maxx = max(src_maxx, tgt_maxx) + 5
        bb_miny = min(src_miny, tgt_miny) - 5
        bb_maxy = max(src_maxy, tgt_maxy) + 5

        # print(bb_minx, bb_miny, bb_maxx, bb_maxy)

        src_img = np.zeros((int(bb_maxy - bb_miny + 1), int(bb_maxx - bb_minx + 1)))
        tgt_img = np.zeros((int(bb_maxy - bb_miny + 1), int(bb_maxx - bb_minx + 1)))

        for p in shifted_pc:
            src_img[int(p[1] - bb_miny), int(p[0] - bb_minx)] = 1

        for p in mask_pc:
            tgt_img[int(p[1] - bb_miny), int(p[0] - bb_minx)] = 1

        rotation_center = [rotation_center[1], rotation_center[0]]

        rotation_center -= np.array([bb_miny, bb_minx])

        best_angle = 0
        best_score = 0
        # for ang in range(0, 360):
        for ang in np.arange(-180, 180, 1):
            score = self.evaluate_rotation(src_img, tgt_img, rotation_center, ang * np.pi / 180)
            # print(ang, score)
            if score > best_score:
                best_score = score
                best_angle = ang * np.pi / 180

        correct_translation = self.get_translation(ori_center, mask_center, translation, best_angle)
        return [correct_translation[0], correct_translation[1], best_angle]

    def get_translation(self, ori_center, mask_center, translation, angle):
        # g = np.array((0, -0.5))
        # c = np.array(ori_center)
        # R_I = np.array([[math.cos(angle) - 1, -math.sin(angle)], [math.sin(angle), math.cos(angle) - 1]])
        # t = np.array(translation)

        g = np.array((0, -0.5))
        c = np.array(ori_center)
        tar_c = np.array(mask_center)
        R = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        t = np.array(translation)
        # print("trans", t)
        # print("c", c)
        # print("tar c", tar_c)
        return np.dot(R, g - c) + tar_c - g

    def evaluate_rotation(self, src_img, tgt_img, center, angle):
        R = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        translation = -np.dot(center, R) + center
        translation = [translation[1], translation[0]]
        translation = np.reshape(translation, [2, 1])
        M = np.concatenate([R, translation], axis=1)

        rotated_img = cv2.warpAffine(src_img, M, (src_img.shape[1], src_img.shape[0]))

        and_img = cv2.bitwise_and(rotated_img, tgt_img)
        or_img = cv2.bitwise_or(rotated_img, tgt_img)
        int_area = np.sum(and_img)
        union_area = np.sum(or_img)
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
        return float(int_area) / union_area

    def get_model_pc(self, letter):
        dir_path = os.path.join(os.path.dirname(__file__), "models")
        return np.loadtxt(os.path.join(dir_path, letter, "pc.txt"))

    def update_pose(self, pose):
        origin = [0, -0.5]
        x, y, _, letter = pose
        x /= 100.0
        y /= 100.0
        color_img, depth_img = self.get_camera_data()
        print("image ready")
        color_heightmap = utils.get_heightmap(
            color_img,
            depth_img,
            self.camera.intrinsics,
            self.cam_pose,
            workspace_limits,
            heightmap_resolution,
            self.camera.camera_to_robot_matrix,
            self.camera.robot_to_camera_matrix,
        )

        outputs = self.predictor(color_heightmap)
        while not self.segmentation_correct(outputs):
            print("wrong segmentation, try again")
            # model.draw_segmentation(color_heightmap, outputs)
            # plt.imshow(color_heightmap)
            # plt.show()
            # Get latest RGB-D image
            color_img, depth_img = self.get_camera_data()
            # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
            color_heightmap = utils.get_heightmap(
                color_img,
                depth_img,
                self.camera.intrinsics,
                self.cam_pose,
                workspace_limits,
                heightmap_resolution,
                self.camera.camera_to_robot_matrix,
                self.camera.robot_to_camera_matrix,
            )
            self.predictor = model.update_score_threshold(-0.05)
            outputs = self.predictor(color_heightmap)
        # model.draw_segmentation(color_heightmap, outputs)
        self.predictor = model.restore_score_threshold(0.7)
        closest_id = None
        closest_dist = float("inf")
        for id, label in enumerate(outputs["instances"].pred_classes):
            # print(id, letter_map[int(label)])
            if letter_map[int(label)] == letter:
                mx, my, Mx, My = outputs["instances"].pred_boxes.tensor.numpy()[id]
                center_x = float(mx + Mx) / 2
                center_y = float(my + My) / 2
                real_x = center_x * heightmap_resolution + workspace_limits[0][0] - origin[0]
                real_y = center_y * heightmap_resolution + workspace_limits[1][0] - origin[1]
                dist = np.linalg.norm(np.array([x, y]) - np.array([real_x, real_y]))
                # print("true", dist)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_id = id
        assert closest_id != None
        mask_pc = self.get_real_point_cloud_from_binary_mask(outputs["instances"].pred_masks[closest_id])
        real_pose = self.get_transformation(np.array(mask_pc), letter)
        return [real_pose[0] * 100, real_pose[1] * 100, real_pose[2], letter]

    def wait_for_execution(self):
        print("Wait for execution...")
        time.sleep(3.0)
        while 1:
            color_img, depth_img = self.camera.get_data()
            if not self.check_tags(color_img):
                time.sleep(2.0)
                continue
            print("tags ready, will check occlusions.")
            ready = True
            color_img, depth_img = self.get_camera_data()
            color_heightmap = utils.get_heightmap(
                color_img,
                depth_img,
                self.camera.intrinsics,
                self.cam_pose,
                workspace_limits,
                heightmap_resolution,
                self.camera.camera_to_robot_matrix,
                self.camera.robot_to_camera_matrix,
            )
            # cv2.imshow("frame", color_heightmap)
            # cv2.waitKey(0)
            gray = cv2.cvtColor(color_heightmap, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
            contours = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            for cnt in contours:
                if cv2.contourArea(cnt) >= 4000:
                    ready = False
                    print("Find large occlusion...")
                    break
            if not ready:
                continue
            else:
                outputs = self.predictor(color_heightmap)
                if not self.segmentation_correct(outputs):
                    continue
                print("Execution ready!")
                return True

    def check_tags(self, color_img):
        cv2.imwrite(os.path.dirname(__file__) + "/real/temp.jpg", color_img)
        p = Popen(
            [os.path.dirname(__file__) + "/real/detect-from-file", os.path.dirname(__file__) + "/real/temp.jpg"],
            stdin=PIPE,
            stdout=PIPE,
            stderr=PIPE,
        )
        output, err = p.communicate()
        tag_info = output.decode("utf-8").split("\n")
        tag_info = [t for t in tag_info if len(list(t)) >= 2]
        for i, info in enumerate(tag_info):
            tag_info[i] = info.split(" ")
        tag_info = np.array(tag_info, dtype=np.float32)
        # print("tag_info", tag_info)
        return tag_info.shape[0] >= 4

    def segmentation_correct(self, outputs, specific_letters=None):
        if specific_letters == None:
            specific_letters = list(self.pool.keys())
        expected_list = []
        for let in specific_letters:
            num = self.pool[let]
            for _ in range(num):
                expected_list.append(let)
        expected_list.sort()
        predicted_list = [
            letter_map[int(outputs["instances"].pred_classes[obj])]
            for obj in range(outputs["instances"].pred_boxes.tensor.numpy().shape[0])
            if letter_map[int(outputs["instances"].pred_classes[obj])] in specific_letters
        ]
        predicted_list.sort()
        return expected_list == predicted_list

    def get_accurate_mask(self, outputs, obj, color_heightmap):
        let = letter_map[int(outputs["instances"].pred_classes[obj])]
        bbox = outputs["instances"].pred_boxes.tensor.numpy()[obj]
        offset = 0
        bbox[0] = max(0, int(bbox[0]) - offset)
        bbox[1] = max(0, int(bbox[1]) - offset)
        bbox[2] = min(color_heightmap.shape[1] - 1, int(bbox[2]) + offset)
        bbox[3] = min(color_heightmap.shape[0] - 1, int(bbox[3]) + offset)
        img = color_heightmap[int(bbox[1]) : int(bbox[3]), int(bbox[0]) : int(bbox[2])]
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # fig = plt.figure()
        # fig.add_subplot(1, 4, 1)
        plt.imshow(img)
        plt.show()
        for c in [0, 1, 2]:
            print(c)
            img_r = img[:, :, c]
            img_r = img_r.flatten()
            img_r = list(img_r)
            # for v in sorted(set(img_r)):
            #     print(v, img_r.count(v))
            VIP_v = [v for v in set(img_r) if img_r.count(v) >= 10]
            VIP_v = sorted(VIP_v)[1:]
            largest_gap = -1
            largest_id = -1
            for id, v in enumerate(VIP_v[:-1]):
                if (VIP_v[id + 1] - v) > largest_gap:
                    largest_gap = VIP_v[id + 1] - v
                    largest_id = id
            if largest_gap > 10:
                cut = VIP_v[largest_id] + largest_gap // 2
                smaller = sum([img_r.count(v) for v in VIP_v[: largest_id + 1]])
                larger = sum([img_r.count(v) for v in VIP_v[largest_id + 1 :]])
                if smaller > larger:
                    for i in range(3):
                        print(c, i)
                        img[:, :, i] = np.where(img[:, :, c] >= cut, 0, img[:, :, i])
                        plt.imshow(img)
                        plt.show()
                else:
                    for i in range(3):
                        print(c, i)
                        img[:, :, i] = np.where(img[:, :, c] <= cut, 0, img[:, :, i])
                        plt.imshow(img)
                        plt.show()
        print("____________________________________________________________________")
        #     fig.add_subplot(1, 4, c + 2)
        #     plt.imshow(img)
        # img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        # plt.show()

    def add_masks(self, letter):
        # Get latest RGB-D image
        color_img, depth_img = self.get_camera_data()
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        color_heightmap = utils.get_heightmap(
            color_img,
            depth_img,
            self.camera.intrinsics,
            self.cam_pose,
            workspace_limits,
            heightmap_resolution,
            self.camera.camera_to_robot_matrix,
            self.camera.robot_to_camera_matrix,
        )
        my_path = os.path.join(os.path.dirname(__file__), "mask_bank", letter)
        if not os.path.exists(my_path):
            os.makedirs(my_path)
        for _, _, files in os.walk(my_path):
            mask_id = str(len(files))
        gray = cv2.cvtColor(color_heightmap, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
        contours = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w <= 20 or h <= 20:
                continue
            roi = color_heightmap[y : y + h, x : x + w]
            roi = cv2.copyMakeBorder(roi, 20, 20, 20, 20, cv2.BORDER_CONSTANT, None, (0, 0, 0))
            cv2.imwrite(os.path.join(my_path, mask_id + ".png"), roi)

    def check_3d_points(self):
        color_img, depth_img = self.get_camera_data()
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        color_heightmap, depth_heightmap, real_depth_heightmap = utils.get_both_heightmaps(
            color_img,
            depth_img,
            self.camera.intrinsics,
            self.cam_pose,
            workspace_limits,
            heightmap_resolution,
            self.camera.camera_to_robot_matrix,
            self.camera.robot_to_camera_matrix,
        )
        np.savetxt("depth.txt", depth_heightmap)
        outputs = self.predictor(color_heightmap)
        num_instances = outputs["instances"].pred_boxes.tensor.numpy().shape[0]
        arr_list = []
        for obj in range(num_instances):
            let = letter_map[int(outputs["instances"].pred_classes[obj])]
            # self.get_accurate_mask(outputs, obj, color_heightmap)
            pc_3d = self.get_real_3d_point_cloud_from_binary_mask(
                outputs["instances"].pred_masks[obj], real_depth_heightmap
            )
            np.savetxt("test.txt", pc_3d)
            # plt.imshow(outputs["instances"].pred_masks[obj])
            # plt.show()
            pc_3d = np.array(pc_3d)
            indices = np.where(pc_3d[:, 2] >= 0.161)
            print(indices)
            x = pc_3d[:, 0]
            y = pc_3d[:, 1]
            plt.scatter(x, y, color="b")
            x = pc_3d[indices, 0]
            y = pc_3d[indices, 1]
            plt.scatter(x, y, color="r")
            plt.show()
            # mask_pc = self.get_real_point_cloud_from_binary_mask(outputs["instances"].pred_masks[obj])
            # pose = self.get_transformation(np.array(mask_pc), let)
            # arr_list.append(tuple(pose + [let]))

    def generate_3d_model(self, letter, pose, maxx=0.172, minn=0.152):

        tx, ty, theta = pose
        # tx *= 5.0
        # ty *= 5.0
        my_path = my_path = os.path.join(os.path.dirname(__file__), "models", letter)
        pc = np.loadtxt(os.path.join(my_path, "pc.txt"))
        num_slices = 5
        slices = [
            np.repeat(i * (maxx - minn) / float(num_slices - 1) + minn, pc.shape[0], axis=0) for i in range(num_slices)
        ]
        z = np.concatenate(slices, axis=0)
        print("slices", z.shape)
        z = z.reshape(-1, 1)
        # print("z", z)
        pc = np.tile(pc, (num_slices, 1))
        print("pc", pc.shape)
        pc = np.concatenate([pc, z], axis=1)
        # print("pc", pc)

        # ax = plt.axes(projection="3d")
        # ax.scatter3D(pc[:, 0], pc[:, 1], pc[:, 2])
        # plt.scatter(pc_3d[:, 0], pc_3d[:, 1], color="r")
        # plt.scatter(rotated_pc[:, 0], rotated_pc[:, 1], color="b")
        # plt.scatter(pc[:, 0], pc[:, 1])
        plt.show()

        rotated_pc = np.zeros(pc.shape)
        center = (0.0, -0.5)
        rotated_pc[:, 0] = (
            math.cos(theta) * (pc[:, 0] - center[0]) - math.sin(theta) * (pc[:, 1] - center[1]) + center[0]
        )
        rotated_pc[:, 1] = (
            math.sin(theta) * (pc[:, 0] - center[0]) + math.cos(theta) * (pc[:, 1] - center[1]) + center[1]
        )
        rotated_pc[:, 2] = pc[:, 2]

        rotated_pc[:, 0] += tx
        rotated_pc[:, 1] += ty

        # ax.scatter3D(pc_3d[:, 0], pc_3d[:, 1], pc_3d[:, 2], c=pc_3d[:, 2], cmap="Greens")
        # plt.scatter(pc_3d[:, 0], pc_3d[:, 1], color="r")
        # plt.scatter(rotated_pc[:, 0], rotated_pc[:, 1], color="b")
        # plt.show()

        # plt.scatter(np.repeat(1, pc_3d.shape[0]), pc_3d[:, 2], color="r")
        # plt.scatter(np.repeat(0, rotated_pc.shape[0]), rotated_pc[:, 2], color="b")
        # plt.show()

        camera_pc = np.transpose(
            np.dot(
                np.transpose(self.cam_pose[0:3, 0:3]),
                (np.transpose(rotated_pc) - np.tile(self.cam_pose[0:3, 3:], (1, rotated_pc.shape[0]))),
            )
        )

        # ax = plt.axes(projection="3d")
        # ax.scatter3D(camera_pts[:, 0], camera_pts[:, 1], camera_pts[:, 2], c="r")
        # ax.scatter3D(camera_pc[:, 0], camera_pc[:, 1], camera_pc[:, 2], c="b")
        # plt.show()
        # plt.scatter(camera_pts[:, 0], camera_pts[:, 1], color="r")
        # plt.scatter(camera_pc[:, 0], camera_pc[:, 1], color="b")
        # plt.show()

        pixel_pts = np.zeros((camera_pc.shape[0], 2))
        pixel_pts[:, 0] = (
            camera_pc[:, 0] * self.camera.intrinsics[0][0] / camera_pc[:, 2] + self.camera.intrinsics[0][2]
        )
        pixel_pts[:, 1] = (
            camera_pc[:, 1] * self.camera.intrinsics[1][1] / camera_pc[:, 2] + self.camera.intrinsics[1][2]
        )

        return pixel_pts, camera_pc, rotated_pc

    def compare_pcs(self):
        color_img, depth_img = self.get_camera_data()
        # plt.imshow(color_img)
        # plt.show()
        # color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        color_heightmap, depth_heightmap, real_depth_heightmap = utils.get_both_heightmaps(
            color_img,
            depth_img,
            self.camera.intrinsics,
            self.cam_pose,
            workspace_limits,
            heightmap_resolution,
            self.camera.camera_to_robot_matrix,
            self.camera.robot_to_camera_matrix,
        )
        plt.imshow(color_heightmap)
        plt.show()
        # np.savetxt("depth.txt", depth_heightmap)
        outputs = self.predictor(color_heightmap)
        model.draw_segmentation(color_heightmap, outputs)
        num_instances = outputs["instances"].pred_boxes.tensor.numpy().shape[0]
        arr_list = []
        for obj in range(num_instances):
            let = letter_map[int(outputs["instances"].pred_classes[obj])]
            binary_mask = outputs["instances"].pred_masks[obj]
            pc_3d = self.get_real_3d_point_cloud_from_binary_mask(binary_mask, real_depth_heightmap)
            mask_pc = self.get_real_point_cloud_from_binary_mask(outputs["instances"].pred_masks[obj])
            # 2D point of the camera
            pc_3d = np.array(pc_3d)
            ax = plt.axes(projection="3d")
            ax.scatter3D(pc_3d[:, 0], pc_3d[:, 1], pc_3d[:, 2])
            plt.show()
            # plt.scatter(pc_3d[:, 0], pc_3d[:, 2])
            # ax = plt.axes()
            # ax.set_aspect("equal", "box")
            # plt.show()
            # plt.scatter(pc_3d[:, 1], pc_3d[:, 2])
            # ax = plt.axes()
            # ax.set_aspect("equal", "box")
            # plt.show()
            camera_pts = np.transpose(
                np.dot(
                    np.transpose(self.cam_pose[0:3, 0:3]),
                    (np.transpose(pc_3d) - np.tile(self.cam_pose[0:3, 3:], (1, pc_3d.shape[0]))),
                )
            )

            # print("cam", camera_pts)

            pixel_pts = np.zeros((camera_pts.shape[0], 2))
            pixel_pts[:, 0] = (
                camera_pts[:, 0] * self.camera.intrinsics[0][0] / camera_pts[:, 2] + self.camera.intrinsics[0][2]
            )
            pixel_pts[:, 1] = (
                camera_pts[:, 1] * self.camera.intrinsics[1][1] / camera_pts[:, 2] + self.camera.intrinsics[1][2]
            )

            # for i in range(pixel_pts.shape[0]):
            #     print("pix", pixel_pts[i, :])

            # binary_mask = np.array(binary_mask)
            # indices = np.where(binary_mask == True)
            # print(indices)
            # seen_pc = np.zeros([len(indices[0]), 2])
            # seen_pc[:, 0] = indices[0][:]
            # seen_pc[:, 1] = indices[1][:]
            pose = self.get_transformation(np.array(mask_pc), let)
            # pose = [0.12979512771039586, -0.09571044009372065, 0.9773843811168246]
            print("pose", pose)
            # dpose = (0.1, 0.1, 0.5)
            # dpose = (0.01, 0.01, 0.1)
            # pose = (
            #     pose[0] + dpose[0],
            #     pose[1] + dpose[1],
            #     pose[2] + dpose[2],
            # )
            # pose = [pose[0], pose[1], -pose[2]]
            model_pc1, camera_pc, rotated_pc = self.generate_3d_model(
                let, pose, maxx=min(0.19, max(pc_3d[:, 2])), minn=min(0.19, max(pc_3d[:, 2])) - 0.01
            )
            # model_pc2 = self.generate_3d_model(let, pose, pc_3d, camera_pts, maxx=0.192, minn=0.132)
            # plt.scatter(pixel_pts[:, 0], pixel_pts[:, 1], color="r")
            # plt.scatter(model_pc2[:, 0], model_pc2[:, 1], color="r")
            # plt.scatter(model_pc1[:, 0], model_pc1[:, 1], color="b")
            # plt.show()
            # pixel_pts[:, 0] = np.round_(pixel_pts[:, 0])
            # pixel_pts[:, 1] = np.round_(pixel_pts[:, 1])
            # model_pc1[:, 0] = np.round_(model_pc1[:, 0])
            # model_pc1[:, 1] = np.round_(model_pc1[:, 1])
            # plt.scatter(pixel_pts[:, 0], pixel_pts[:, 1], color="r")
            # plt.scatter(model_pc1[:, 0], model_pc1[:, 1], color="b")
            # plt.show()
            refinement = ICP(model_pc1, pixel_pts)
            best_pix_pc = refinement.src_pc
            best_camera_pc = np.zeros([best_pix_pc.shape[0], 3])
            best_camera_pc[:, 0] = (
                (best_pix_pc[:, 0] - self.camera.intrinsics[0][2]) * camera_pc[:, 2] / self.camera.intrinsics[0][0]
            )
            best_camera_pc[:, 1] = (
                (best_pix_pc[:, 1] - self.camera.intrinsics[1][2]) * camera_pc[:, 2] / self.camera.intrinsics[1][1]
            )
            best_camera_pc[:, 2] = camera_pc[:, 2]

            surface_pts = np.transpose(
                np.dot(self.cam_pose[0:3, 0:3], np.transpose(best_camera_pc))
                + np.tile(self.cam_pose[0:3, 3:], (1, best_camera_pc.shape[0]))
            )
            # print("suf")
            # plt.scatter(surface_pts[:, 0], surface_pts[:, 1], color="b")
            # plt.scatter(pc_3d[:, 0], pc_3d[:, 1], color="r")
            # plt.show()
            pose, final_pc = self.compute_refined_pose(pose, rotated_pc[:, :2], surface_pts[:, :2])
            print("cmp")
            plt.scatter(final_pc[:, 0], final_pc[:, 1], color="b")
            plt.scatter(pc_3d[:, 0], pc_3d[:, 1], color="r")
            plt.show()
            print("pose", pose)
            return pose

    def compute_refined_pose(self, pose, src_pc, tgt_pc):
        # plt.scatter(src_pc[:, 0], src_pc[:, 1], color="r")
        # plt.scatter(tgt_pc[:, 0], tgt_pc[:, 1], color="b")
        # plt.show()
        src_center = np.array(
            [
                np.average(src_pc[:, 0]),
                np.average(src_pc[:, 1]),
            ]
        )

        tgt_center = np.array(
            [
                np.average(tgt_pc[:, 0]),
                np.average(tgt_pc[:, 1]),
            ]
        )

        norm_src = src_pc - src_center
        norm_tgt = tgt_pc - tgt_center

        W = np.dot(np.transpose(norm_tgt), norm_src)

        u, _, vh = np.linalg.svd(W)
        # print(u)
        # print(s)
        # print(vh)
        R = np.dot(u, vh)
        # print("R", R)
        t = tgt_center - np.dot(R, src_center)
        src_pc = np.transpose(np.dot(R, np.transpose(norm_src))) + tgt_center
        # print("t", t)
        theta = math.asin(R[1, 0])
        new_pose = (pose[0] + t[0], pose[1] + t[1], pose[2] + theta)
        # plt.scatter(src_pc[:, 0], src_pc[:, 1], color="b")
        # plt.scatter(tgt_pc[:, 0], tgt_pc[:, 1], color="r")
        # plt.show()
        return new_pose, src_pc


if __name__ == "__main__":
    P = Perception("R")
    # P.generate_3d_model("R")
    # P.compare_pcs()
    # P.check_3d_points()
    # P.save_pose("0")
    # P.add_masks("0")
    P.get_arrangement(purpose="S")
    # P.get_arrangement(purpose="G")
