import numpy as np

class RoverState():
    def __init__(self):
        self.start_time = None # To record the start time of navigation
        self.total_time = None # To record total duration of navigation
        self.img = None # Current camera image
        self.pos = None # Current position (x,y)
        self.yaw = None # Current yaw
        self.pitch = None
        self.roll = None
        self.vel = None # Current velocity
        self.steer = 0 # Current steering angle
        self.throttle = 0
        self.brake = 0
        self.nav_angles = None # Angles of navigable terrain pixels
        self.nav_dists = None # Distances of navigable terrain pixels
        self.ground_truth = None # Ground truth worldmap ---> set to groound_truth_3d
        self.mode = 'forward' # Current mode
        self.throttle_set = 0.2 # Throttle setting when accelerating
        self.brake_set = 10 # Brake setting when braking
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!
        self.stop_forward = 50 # Threshold to initiate stopping
        self.go_forward = 500 # Threshold to go forward again
        self.max_vel = 2 # m/s
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode  
        self.vision_image = np.zeros((160,320,3), dtype=np.float)
        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200,200,3),dtype=np.float)
        self.sample_pos = None # To store the actual sample positions
        self.samples_found = 0 # Count of samples
        self.near_sample = False # Set to true if within sample
        self.pick_up = False # Set to True to trigger pickup