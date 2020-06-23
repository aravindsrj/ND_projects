import numpy as np
import cv2

def perspect_transform_comp(image, dst_size, bottom_offset):

    source = np.float32([[14 ,140 ], [118 ,96 ], [200 ,96 ], [301 ,140 ]])
    destination = np.float32([[image.shape[1]/2-dst_size , image.shape[0]-bottom_offset], 
    [image.shape[1]/2-dst_size , image.shape[0]-bottom_offset-2*dst_size], 
    [image.shape[1]/2+dst_size , image.shape[0]-bottom_offset-2*dst_size], 
    [image.shape[1]/2+dst_size ,image.shape[0]-bottom_offset]])


    M = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(image, M, (image.shape[1], image.shape[0]))
    return warped


def color_thresh(img,rgb_thresh):
    b_img = np.zeros_like(img[:,:,0])
    color_select = (img[:,:,0]>rgb_thresh[0]) & (img[:,:,1]>rgb_thresh[1]) & (img[:,:,2]>rgb_thresh[2])
    b_img[color_select] = 1
    return b_img

def rover_coords(binary_image):
    # converting to rover coordinates
    ypos, xpos = binary_image.nonzero()
    xpixel = (binary_image.shape[0] - ypos).astype(np.float)
    ypixel = (binary_image.shape[1]/2 - xpos).astype(np.float)
    return(xpixel,ypixel)

def rotate_pix(xpix,ypix,yaw):
    # yaw angle is recorded in degrees so first convert to radians
    yaw_rad = yaw * np.pi / 180
    x_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    y_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    return(x_rotated,y_rotated)

def translate_pix(x_rotated, y_rotated, xpos, ypos, worldsize, scale):
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

def to_polar_coords(xpix, ypix):
    dist = np.sqrt(xpix**2 + ypix**2)
    angle = np.arctan2(ypix,xpix)
    return dist,angle

def perception_step(Rover):
    return Rover