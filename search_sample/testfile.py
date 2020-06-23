#! /usr/bin/env python
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

filename = '/home/aravind/Documents/Udacity - Robotics Software Engineer Nanodegree/Udacity - Robotics Software Engineer v1.0.0/project1/RoboND-Rover-Project-master/test_dataset/IMG/robocam_2017_05_02_11_16_21_421.jpg'
image = mpimg.imread(filename)

red_channel = np.copy(image)

print(image.dtype, image.shape)

red_channel[:,:,[1,2]] = 0

plt.imshow(red_channel)
plt.show()