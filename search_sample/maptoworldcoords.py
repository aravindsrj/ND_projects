'''Converts map to world coordinates
'''

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
from supporting_fns import perspect_transform_comp, color_thresh, rover_coords

def rotate_pix(xpix,ypix,yaw):
    # yaw angle is recorded in degrees so first convert to radians
    yaw_rad = yaw * np.pi / 180
    x_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    y_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    return(x_rotated,y_rotated)

def translate_pix(x_rotated, y_rotated, xpos, ypos, worldsize, scale):
    # Assume a scale factor of 10 between world space pixels and rover space pixels

    # Perform translation and convert to integer since pixel values can't be float
    x_world = np.int_(xpos + (x_rotated / scale))
    y_world = np.int_(ypos + (y_rotated / scale))

    #world_size = 200 # 200 x 200
    x_pix_world = np.clip(x_world,0,worldsize)
    y_pix_world = np.clip(y_world,0,worldsize)
    return(x_pix_world,y_pix_world)

def pix_to_world(xpix, ypix, xpos, ypos, yaw, worldsize, scale):
    x_rot,y_rot = rotate_pix(xpix,ypix,yaw)
    x_pix_world, y_pix_world = translate_pix(x_rot,y_rot,xpos,ypos,worldsize,scale)
    return(x_pix_world,y_pix_world)


filename = '/home/aravind/Documents/Udacity - Robotics Software Engineer Nanodegree/Udacity - Robotics Software Engineer v1.0.0/project1/IMG/robocam_2020_03_21_16_33_33_470.jpg'
image = mpimg.imread(filename)

# Rover yaw values will come as floats from 0 to 360
rover_yaw = np.random.random(1)*360

# Generate a random rover position in world coords
# Position values will range from 20 to 180 to 
# avoid the edges in a 200 x 200 pixel world
rover_xpos = np.random.random(1)*160 + 20
rover_ypos = np.random.random(1)*160 + 20

world_map = np.zeros((200,200))
scale = 10

# This line conducts the perspective tranform of the image
warped = perspect_transform_comp(image, dst_size=5, bottom_offset=6)

# This line conducts the thresholding of the image
colorsel = color_thresh(warped, rgb_thresh=(160,160,160))


# This line take the points and convert them with respect to the rover as the origin
xpix, ypix = rover_coords(colorsel)

# This line maps the traversable area into the world map
x_world, y_world = pix_to_world(xpix, ypix, rover_xpos, rover_ypos, rover_yaw, world_map.shape[0],scale)

# Add pixel positions to worldmap
world_map[y_world,x_world] += 1
print('Xpos =', rover_xpos, 'Ypos =', rover_ypos, 'Yaw =', rover_yaw)

# Plot the map in rover-centric coordinates
f, (ax1,ax2) = plt.subplots(1,2, figsize=(12, 7))
f.tight_layout()
ax1.plot(xpix, ypix, '.')
ax1.set_title('Rover Space', fontsize=40)
ax1.set_ylim(-160, 160)
ax1.set_xlim(0, 160)
ax1.tick_params(labelsize=20)

ax2.imshow(world_map, cmap='gray')
ax2.set_title('World Space', fontsize=40)
ax2.set_ylim(0, 200)
ax2.tick_params(labelsize=20)
ax2.set_xlim(0, 200)

plt.subplots_adjust(left=0.1, right=1, top=0.9, bottom=0.1)
plt.show() # Uncomment if running on your local machine