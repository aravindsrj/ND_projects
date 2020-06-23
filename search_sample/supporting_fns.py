'''Supplementary functions
'''

import numpy as np
import cv2
import base64
import time
from PIL import Image
from io import BytesIO, StringIO

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

def convert_to_float(string_to_convert):
    float_value = np.float(string_to_convert.replace(',','.'))
    return float_value

def update_rover(Rover, data):
    if Rover.start_time == None:
        Rover.start_time = time.time()
        Rover.total_time = 0
        samples_xpos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_x"].split(';')])
        samples_ypos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_y"].split(';')])
        Rover.samples_pos = (samples_xpos, samples_ypos)
        Rover.samples_to_find = np.int(data["sample_count"])
    else:
        tot_time = time.time() - Rover.start_time
        if np.isfinite(tot_time):
            Rover.total_time = tot_time
    print(data.keys())
    Rover.vel = convert_to_float(data["speed"])
    Rover.pos = [convert_to_float(pos.strip()) for pos in data["position"].split(';')]
    Rover.yaw = convert_to_float(data["yaw"])
    Rover.pitch = convert_to_float(data["pitch"])
    Rover.roll = convert_to_float(data["roll"])
    Rover.throttle = convert_to_float(data["throttle"])
    Rover.steer = convert_to_float(data["steering_angle"])
    Rover.near_sample = np.int(data["near_sample"])
    Rover.picking_up = np.int(data["picking_up"])
    Rover.samples_collected = Rover.samples_to_find - np.int(data["sample_count"])

    print('speed =',Rover.vel, 'position =',Rover.pos, 'throttle =',
    Rover.throttle, 'steer_angle =', Rover.steer, 'near_sample:', Rover.near_sample, 
    'picking_up:', data["picking_up"], 'sending pickup:', Rover.send_pickup, 
    'total time:', Rover.total_time, 'samples remaining:', data["sample_count"], 
    'samples collected:', Rover.samples_collected)
    
    # Get the current image from the center camera of the rover
    imgString = data["image"]
    image = Image.open(BytesIO(base64.b64decode(imgString)))
    Rover.img = np.asarray(image)

    return Rover, image

# Define a function to create display output given worldmap results
def create_output_images(Rover):
    # Create a scaled map for plotting and clean up obs/nav pixels a bit
    if np.max(Rover.worldmap[:,:,2]) > 0:
        nav_pix = Rover.worldmap[:,:,2] > 0
        navigable = Rover.worldmap[:,:,2] * (255 / np.mean(Rover.worldmap[nav_pix, 2]))
    else:
        navigable = Rover.worldmap[:,:,2]
    
    if np.max(Rover.worldmap[:,:,0]) > 0:
        obs_pix = Rover.worldmap[:,:,0] > 0
        obstacle = Rover.worldmap[obs_pix] * (255 / np.mean(Rover.worldmap[obs_pix, 0]))
    else:
        obstacle = Rover.worldmap[:,:,0]
    
    likely_nav = navigable >= obstacle
    obstacle[likely_nav] = 0
    plotmap = np.zeros_like(Rover.worldmap)
    plotmap[:,:,0] = obstacle
    plotmap[:,:,2] = navigable
    plotmap = plotmap.clip(0,255)

    # Overlay obstacle and navigable terrain map with ground truth map
    map_add = cv2.addWeighted(plotmap, 1, Rover.ground_truth, 0.5, 0)

    # Check whether any rock detections are present in worldmap
    rock_world_pos = Rover.worldmap[:,:,1].nonzero()
    # If there are, we'll step through the known sample positions
    # to confirm whether detections are real
    samples_located = 0

    if rock_world_pos[0].any():

        rock_size = 2
        for idx in range(len(Rover.samples_pos[0])):
            test_rock_x = Rover.samples_pos[0][idx]
            test_rock_y = Rover.samples_pos[1][idx]
            rock_samples_dists = np.sqrt((test_rock_x - rock_world_pos[1])**2 + \
                                    (test_rock_y - rock_world_pos[0])**2)
            # If rocks were detected within 3 meters of known sample positions
            # consider it a success and plot the location of the known
            # sample on the map
            if np.min(rock_samples_dists) < 3:
                samples_located += 1
                map_add[test_rock_y-rock_size:test_rock_y+rock_size,
                test_rock_x-rock_size:test_rock_x+rock_size, :] = 255
    
    # Calculate some statistics on the map results
    # First get the total number of pixels in the navigable terrain map
    tot_nav_pix = np.float(len(plotmap[:,:,2].nonzero()[0]))
    # Next figure out how many of those correspond to ground truth pixels
    good_nav_pix = np.float(len(((plotmap[:,:,2] > 0) & (Rover.ground_truth[:,:,1] > 0)).nonzero()[0]))
    # Next find how many do not correspond to ground truth pixels
    bad_nav_pix = np.float(len(((plotmap[:,:,2] > 0) & (Rover.ground_truth[:,:,1] == 0)).nonzero()[0]))



       