'''Color threshold
Takes an image and threshold values as input
Returns binary thresholded image
'''

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

filename = '/home/aravind/Documents/Udacity - Robotics Software Engineer Nanodegree/Udacity - Robotics Software Engineer v1.0.0/project1/IMG/robocam_2020_03_21_16_33_33_470.jpg'

image = mpimg.imread(filename)

def color_thresh(img,rgb_thresh):
    b_img = np.zeros_like(img[:,:,0])
    color_select = (img[:,:,0]>rgb_thresh[0]) & (img[:,:,1]>rgb_thresh[1]) & (img[:,:,2]>rgb_thresh[2])
    b_img[color_select] = 1
    return b_img

rgb_thresh = (160,160,160)
bin_img = color_thresh(image,rgb_thresh)

f, (ax1,ax2) = plt.subplots(1,2, figsize=(21,7),sharey=True)
f.tight_layout()

ax1.imshow(image)
ax1.set_title('Org Image', fontsize=40)

ax2.imshow(bin_img,cmap='gray')
ax2.set_title('Result',fontsize=40)
plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
plt.show()

plt.imshow(image)
plt.show()