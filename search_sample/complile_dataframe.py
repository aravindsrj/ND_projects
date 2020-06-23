'''Compiling everything
'''

import pandas as pd
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2
from supporting_fns import *
from moviepy.editor import VideoFileClip, ImageSequenceClip
from IPython.display import HTML


fields = ['Path','SteerAngle','Throttle','Speed','X_Position','Y_Position',
    'Pitch','Yaw','Roll']
df = pd.read_csv('./robot_log.csv',delimiter=';',decimal='.',header=0)
csv_img_list = df["Path"].tolist() # Create list of image path names

# Read in ground truth map and create a 3-channel image with it
ground_truth = mpimg.imread('./RoboND-Rover-Project-master/calibration_images/map_bw.png')
ground_truth_3d = np.dstack((ground_truth*0,ground_truth*255,ground_truth*0)).astype(np.float)

# Creating a class to be the data container
# Will read in saved data from csv file and populate this object
# Worldmap is instantiated as 200 x 200 grids corresponding 
# to a 200m x 200m space (same size as the ground truth map: 200 x 200 pixels)
# This encompasses the full range of output position values in x and y from the sim
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

def process_image(img):
    warped = perspect_transform_comp(img,dst_size=5,bottom_offset=6)
    colorsel = color_thresh(img,rgb_thresh=(160,160,160))
    xpix,ypix = rover_coords(colorsel)
    xpix,ypix = pix_to_world(xpix,ypix,data.xpos[data.count],
                data.ypos[data.count], data.yaw[data.count], 
                data.worldmap.shape[0],10)
    data.worldmap[ypix,xpix,0] += 1
    
    
    # Creating a mosaic
    output_image = np.zeros((img.shape[0] + data.worldmap.shape[0],
                    img.shape[1]*2, 3))
    cs = np.zeros((img.shape[0],img.shape[1],3)).astype(np.uint8)
    cs[:,:,0] = cs[:,:,1] = cs[:,:,2] = colorsel*255 
    # plt.imshow(cs)
    # plt.show()
    output_image[0:img.shape[0], 0:img.shape[1]] = img
    output_image[0:img.shape[0], img.shape[1]:] = warped
    output_image[img.shape[0]:img.shape[0]+cs.shape[0], img.shape[1]:output_image.shape[1]] = cs

    # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(data.worldmap, 10, data.ground_truth, 0.5,0)

    # Flip map overlay so y-axis points upwards and add to output image
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)

    cv2.putText(output_image,"Populate this image with your analyses to make a video",
        (20,20), cv2.FONT_HERSHEY_COMPLEX,0.4,(255,255,255),1)
    if data.count < len(data.images) - 1:
        data.count += 1

    return output_image


# Video stuff
output = './output/test_mapping.mp4'
clip = ImageSequenceClip(data.images, fps=60) # simulator speed is 25fps
new_clip = clip.fl_image(process_image)
new_clip.write_videofile(output, audio=False)

HTML("""
<video width="960" height="540" controls>
  <source src="{0}">
</video>
""".format(output))