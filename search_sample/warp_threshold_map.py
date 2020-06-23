import matplotlib.pyplot as plt
import matplotlib.image as mpimg
#%matplotlib inline
import numpy as np
import cv2
from supporting_fns import perspect_transform_comp, color_thresh

filename = '/home/aravind/Documents/Udacity - Robotics Software Engineer Nanodegree/Udacity - Robotics Software Engineer v1.0.0/project1/IMG/robocam_2020_03_21_16_33_33_470.jpg'
image = mpimg.imread(filename)

warped = perspect_transform_comp(image, dst_size=5, bottom_offset=6)
colorsel = color_thresh(warped, rgb_thresh=(160,160,160))

#plt.imshow(colorsel, cmap='gray')


def rover_coords(binary_image):
    ypos, xpos = binary_image.nonzero()
    xpixel = (binary_image.shape[0] - ypos).astype(np.float)
    ypixel = (binary_image.shape[1]/2 - xpos).astype(np.float)
    return(xpixel,ypixel)


xpos, ypos = rover_coords(colorsel)
plt.plot(xpos,ypos,'.')
plt.xlim(0,160)
plt.ylim(-160,160)
plt.title('Rover-centric map', fontsize=20)
plt.show()