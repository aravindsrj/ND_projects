#! /usr/bin/env python
'''
The input images are not of correct dimensions
'''

# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from time import time
import cv2

image_location = '1.png'
depth_map_location = 'apple_depth_image.png'

depth_map_o = mpimg.imread(depth_map_location)
depth_map = cv2.cvtColor(depth_map_o, cv2.COLOR_RGBA2GRAY)

image = mpimg.imread(image_location)
print(image.shape)

start_time = time()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

rows, cols = depth_map.shape
# print(depth_map.shape)

# Send pixels not correctly mapped to the back
# Use 1.1 to send the poorly mapped pixels back 10 percent 
# farther than the farthest correctly mapped point
max_val = depth_map.max() * 1.1

count = 0
# non_zero_count = 0

# Cut down pixels for time purpose 
# Computations need to be under 30s
pixel_cut = 3

# print(depth_map[400,400])

# print(depth_map[10,10])
for x in range(cols):
    if x == 300:
        break
    for y in range(rows):
        if (x%pixel_cut == 0 and y%pixel_cut == 0):
            count+= 1

            # TODO 
            # Get point color for pixel
            # Format as tuple: (R,G,B)
            pixel_color = tuple(image[y][x][:3])
            

            # TODO
            # Get point depth for each particle
            depth = depth_map[y,x]

            # if depth != 1:
            #     non_zero_count += 1

            if depth == 0:
                depth = max_val
            
            ax.scatter(x, depth, y, c=pixel_color, marker='o')

            if count % 100 == 0:
                # print("Count = {}".format(count))
                print("x = {}".format(x))
    
# Axis labels
ax.set_xlabel('Width')
ax.set_ylabel('Depth')
ax.set_zlabel('Height')

plt.gca().invert_zaxis()

###########################################
# Play with me to change view rotation!
elevation = 30 # Up/Down
azimuth = 300 # Left/Right
###########################################

ax.view_init(elevation, azimuth)

print("Outputted {} of {} points".format(count,6552))
print("Results produced in {:04.2f} seconds".format(time()-start_time))  
# print("Non zero count = ", non_zero_count)
plt.show()    
