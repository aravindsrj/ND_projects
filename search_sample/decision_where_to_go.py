'''Returns distance and angle of best direction to follow
'''
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import pandas as pd
import cv2
from supporting_fns import perspect_transform_comp, color_thresh, rover_coords

def to_polar_coords(xpix, ypix):
    dist = np.sqrt(xpix**2 + ypix**2)
    angle = np.arctan2(ypix,xpix)
    return dist,angle

filename = filename = '/home/aravind/Documents/Udacity - Robotics Software Engineer Nanodegree/Udacity - Robotics Software Engineer v1.0.0/project1/IMG/robocam_2020_03_21_16_33_33_470.jpg'
image = mpimg.imread(filename)

warped = perspect_transform_comp(image, dst_size=5, bottom_offset=6)
colorsel = color_thresh(warped, rgb_thresh=(160,160,160))

xpix,ypix = rover_coords(colorsel)
# dist,angles = to_polar_coords(xpix,ypix)
# avg_angle = np.mean(angles)

# fig = plt.figure(figsize=(12,9))
# plt.subplot(221)
# plt.imshow(image)
# plt.subplot(222)
# plt.imshow(warped)
# plt.subplot(223)
# plt.imshow(colorsel,cmap='gray')
# plt.subplot(224)
# plt.plot(xpix,ypix,'.')
# plt.ylim(-160,160)
# plt.xlim(0,160)
# arrow_length = 100
# x_arrow = arrow_length * np.cos(avg_angle)
# y_arrow = arrow_length * np.sin(avg_angle)
# plt.arrow(0,0,x_arrow,y_arrow,color='red',zorder=2,head_width=10,width=2)
# plt.show()

df = pd.read_csv('./robot_log.csv',delimiter=';',decimal='.',header=0)
csv_img_list = df["Path"].tolist() # Create list of image path names

ground_truth = mpimg.imread('./RoboND-Rover-Project-master/calibration_images/map_bw.png')
ground_truth_3d = np.dstack((ground_truth*0,ground_truth*255,ground_truth*0)).astype(np.float)

class Databucket():
    def __init__(self):
        self.images = csv_img_list
        self.xpos = df["X_Position"].values
        self.ypos = df["Y_Position"].values
        self.yaw = df["Yaw"].values
        self.count = 0
        self.worldmap = np.zeros((200,200,3)).astype(np.float)
        self.ground_truth = ground_truth_3d # Ground truth world map

data = Databucket()


