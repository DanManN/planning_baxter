# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2017 Intel Corporation. All Rights Reserved.


#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs

# Import Numpy for easy array manipulation
import numpy as np

# Import OpenCV for easy image rendering
import cv2
import sys
import atexit
from typing import Tuple
import time
from subprocess import Popen, PIPE
from scipy import optimize
import os

sys.path.insert(
    0, 
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    )
from constants import tag_loc_robot, tag_loc_robot_height

# Estimate rigid transform with SVD (from Nghia Ho)


def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]  # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1))  # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB)  # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:  # Special reflection case
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t


def get_rigid_transform_error(z_scale):
    global measured_pts, observed_pts, observed_pix, world2camera, camera_intrinsics

    # Apply z offset and compute new observed points using camera intrinsics
    observed_z = observed_pts[:, 2:] * z_scale
    observed_x = np.multiply(observed_pix[:, [0]] - camera_intrinsics[0][2], observed_z / camera_intrinsics[0][0])
    observed_y = np.multiply(observed_pix[:, [1]] - camera_intrinsics[1][2], observed_z / camera_intrinsics[1][1])
    new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

    # Estimate rigid transform between measured points and new observed points
    R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
    t.shape = (3, 1)
    world2camera = np.concatenate((np.concatenate((R, t), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

    # Compute rigid transform error
    registered_pts = np.dot(R, np.transpose(measured_pts)) + np.tile(t, (1, measured_pts.shape[0]))
    error = np.transpose(registered_pts) - new_observed_pts
    error = np.sum(np.multiply(error, error))
    rmse = np.sqrt(error / measured_pts.shape[0])
    return rmse


# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
# config.enable_stream(rs.stream.depth, 1920, 1080, rs.format.z16, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)
# cfg = pipeline.start() # Start pipeline and get the configuration it found
p = profile.get_stream(rs.stream.color) # Fetch stream profile for depth stream
intr = p.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics

print('color')
print( intr.ppx)
print( intr.ppy)
print( intr.fx)
print( intr.fy)

p = profile.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
intr = p.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
print('depth')
print( intr.ppx)
print( intr.ppy)
print( intr.fx)
print( intr.fy)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1  # 1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# tag_loc_robot = {
#     105: (929.5 / 1000, -662 / 1000),
#     107: (723 / 1000, 49.5 / 1000),
#     108: (665 / 1000, -643 / 1000),
#     109: (969.6 / 1000, 38.8 / 1000)
# }

# tag_loc_robot_height = {
#     105: 1.145,
#     107: 1.145,
#     108: 1.145,
#     109: 1.145
# }


measured_pts = []
observed_pts = []
observed_pix = []
world2camera = np.eye(4)
camera_intrinsics = None

decimate = rs.decimation_filter()
hole = rs.hole_filling_filter(2)
spat_filter = rs.spatial_filter()


class Camera(object):
    """Customized realsense camera for VPG work"""

    def __init__(self, need_cali = True):
        atexit.register(self.stop_streaming)
        # Extra sleep time for stabilizing camera configs
        time.sleep(2)
        # self.intrinsics = np.array([[922.292, 0, 644.33], [0, 922.554, 358.628], [0, 0, 1]])
        self.intrinsics = np.array([[1385.3297119140625, 0, 970.3392944335938], [0, 1385.8759765625, 536.2879028320312], [0, 0, 1]])
        self.camera_to_robot_matrix = None
        self.robot_to_camera_matrix = None
        self.calculate_transformation()
        if need_cali:
            self.calibrate()

    def stop_streaming(self):
        """Release camera resource"""
        pipeline.stop()

    def get_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns:
            np.ndarray: 1280x720 color image
            np.ndarray: 1280x720 depth image
        """
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        # Validate that both frames are valid
        assert color_frame
        assert aligned_depth_frame

        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        # post-processing depth image
        # aligned_depth_frame = decimate.process(aligned_depth_frame)
        aligned_depth_frame = spat_filter.process(aligned_depth_frame)
        aligned_depth_frame = hole.process(aligned_depth_frame)
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_image = depth_image * depth_scale

        # color_image = cv2.imread(os.path.dirname(__file__) + "/temp_s.jpg")

        return color_image, depth_image

    def read_tags(self):
        while 1:
            color_img, _ = self.get_data()
            cv2.imwrite(os.path.dirname(__file__) + "/temp.jpg", color_img)
            # cv2.imshow('frame', color_img)
            # cv2.waitKey(0)
            p = Popen(
                [os.path.dirname(__file__) + "/estimate3d-gui-from-file", os.path.dirname(__file__) + "/temp.jpg"],
                stdin=PIPE,
                stdout=PIPE,
                stderr=PIPE,
            )
            output, err = p.communicate()
            # print('output', output)
            tag_info = output.decode("utf-8").split("\n")
            tag_info = tag_info[3:]
            print('tag_info', tag_info)
            tag_info = [t.split()[:3] for t in tag_info 
            if (len(t.split())>=3) and (int(t.split()[0]) in tag_loc_robot.keys())]
            
            # for i, info in enumerate(tag_info):
            #     tag_info[i] = info.split(" ")
            # print("tag", tag_info)
            tag_info = np.array(tag_info, dtype=np.float32)
            print("tag", tag_info)
            if tag_info.shape == (4, 3):
                return tag_info
            else:
                print('wrong tag info, try again')

    def calculate_transformation(self):
        # color_img, _depth_img = self.get_data()
        # cv2.imshow('calc', color_img)
        # cv2.waitKey(0)
        tag_loc_camera = self.read_tags()
        self.camera_to_robot_matrix = cv2.getPerspectiveTransform(
            np.float32([tag[1:] for tag in tag_loc_camera]),
            np.float32([tag_loc_robot[tag[0]] for tag in tag_loc_camera]),
        )
        self.robot_to_camera_matrix = cv2.getPerspectiveTransform(
            np.float32([tag_loc_robot[tag[0]] for tag in tag_loc_camera]),
            np.float32([tag[1:] for tag in tag_loc_camera]),
        )

    def camera_to_robot(self, pt):
        camera_pt = np.array([pt[0], pt[1], 1])
        robot_pt = np.dot(self.camera_to_robot_matrix, camera_pt)
        robot_pt = np.array([robot_pt[0], robot_pt[1]]) / robot_pt[2]
        return robot_pt

    def robot_to_camera(self, pt):
        robot_pt = np.array([pt[0], pt[1], 1])
        camera_pt = np.dot(self.camera_to_robot_matrix, robot_pt)
        camera_pt = np.array([camera_pt[0], camera_pt[1]]) / camera_pt[2]
        return camera_pt

    def calibrate(self):
        global measured_pts, observed_pts, observed_pix, world2camera, camera_intrinsics
        measured_pts = []
        observed_pts = []
        observed_pix = []
        world2camera = np.eye(4)
        camera_intrinsics = self.intrinsics
        _, depth_img = self.get_data()
        # cv2.imshow('calib', color_img)
        # cv2.waitKey(0)
        tag_loc_camera = self.read_tags()
        for tag in tag_loc_camera:
            checkerboard_pix = [int(tag[1]), int(tag[2])]
            checkerboard_z = depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
            checkerboard_x = np.multiply(
                checkerboard_pix[0] - self.intrinsics[0][2], checkerboard_z / self.intrinsics[0][0]
            )
            checkerboard_y = np.multiply(
                checkerboard_pix[1] - self.intrinsics[1][2], checkerboard_z / self.intrinsics[1][1]
            )
            observed_pts.append([checkerboard_x, checkerboard_y, checkerboard_z])
            measured_pts.append([tag_loc_robot[tag[0]][0], tag_loc_robot[tag[0]][1], tag_loc_robot_height[tag[0]]])
            observed_pix.append(checkerboard_pix)
        measured_pts = np.asarray(measured_pts)
        observed_pts = np.asarray(observed_pts)
        observed_pix = np.asarray(observed_pix)

        print("Calibrating...")
        z_scale_init = 1
        optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method="Nelder-Mead")
        camera_depth_offset = optim_result.x

        # Save camera optimized offset and camera pose
        print("Saving...")
        np.savetxt(os.path.dirname(__file__) + "/camera_depth_scale.txt", camera_depth_offset, delimiter=" ")
        get_rigid_transform_error(camera_depth_offset)
        camera_pose = np.linalg.inv(world2camera)
        np.savetxt(os.path.dirname(__file__) + "/camera_pose.txt", camera_pose, delimiter=" ")
        print("Done.")


class Camera_no_tag(object):
    """get frame without detecting the tags"""

    def __init__(self):
        atexit.register(self.stop_streaming)
        # Extra sleep time for stabilizing camera configs
        time.sleep(2)
        self.intrinsics = np.array([[922.292, 0, 644.33], [0, 922.554, 358.628], [0, 0, 1]])
        self.camera_to_robot_matrix = None
        self.robot_to_camera_matrix = None

    def stop_streaming(self):
        """Release camera resource"""
        pipeline.stop()

    def get_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns:
            np.ndarray: 1280x720 color image
            np.ndarray: 1280x720 depth image
        """
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        # Validate that both frames are valid
        assert color_frame
        assert aligned_depth_frame

        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        # post-processing depth image
        # aligned_depth_frame = decimate.process(aligned_depth_frame)
        aligned_depth_frame = spat_filter.process(aligned_depth_frame)
        aligned_depth_frame = hole.process(aligned_depth_frame)
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_image = depth_image * depth_scale

        return color_image, depth_image


# depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
# print(depth_profile)
# depth_intrinsics = depth_profile.get_intrinsics()
# print(depth_intrinsics)
# w, h = depth_intrinsics.width, depth_intrinsics.height
# print(w, h)
# pipeline.stop()
