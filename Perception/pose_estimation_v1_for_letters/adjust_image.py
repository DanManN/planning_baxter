import cv2
import os
from matplotlib import pyplot as plt
import numpy as np

from real.camera import Camera
# camera = Camera()
# color_image, _ = camera.get_data()
# # color_image = cv2.imread(os.path.dirname(os.path.abspath(__file__)) + "/real/temp.jpg")
# plt.imshow(color_image)
# plt.show()
def adjust_image(color_image):
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)
    # np.where((hsv_image[:, :, 0]<=127, 2*hsv_image[:, :, 0], 255))
    s = hsv_image[:,:,1]
    scale = 1.6
    # s = 2*s
    s = np.where((scale*s)<=255, scale*s, 255)
    hsv_image[:,:,1] = s
    # for i in range(hsv_image.shape[0]):
    #     for j in range(hsv_image.shape[1]):
    #         hsv_image[i, j, 1] = min(int(1.625*hsv_image[i, j, 1]), 255)
    color_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
    scalar = 1.4
    color_image = np.where((scalar*color_image)<=255, scalar*color_image, 255)
    # scalar = 0.9
    # color_image[:,:,1] = np.where((scalar*color_image[:,:,1])>=0, scalar*color_image[:,:,1], 0)
    scalar = 1.2
    color_image[:, :, :1] = np.where((scalar*color_image[:, :, :1])<=255, scalar*color_image[:, :, :1], 255)
    color_image[:, :, 2] = np.where((color_image[:, :, 2]/scalar)>=0, color_image[:, :, 2]/scalar, 0)
    # color_image[:, :, 1] = color_image[:, :, 1]/1.1
    color_image = color_image.astype(np.uint8)
    # plt.imshow(color_image)
    # plt.show()
    # plt.imsave(os.path.dirname(os.path.abspath(__file__)) + "/real/temp_s.jpg", color_image)
    return color_image

if __name__ == '__main__':
    camera = Camera()
    color_image, _ = camera.get_data()
    # color_image = cv2.imread(os.path.dirname(os.path.abspath(__file__)) + "/real/temp.jpg")
    # plt.imshow(color_image)
    # plt.show()
    color_image = adjust_image(color_image)
    plt.imshow(color_image)
    plt.show()
    