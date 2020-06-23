'''Controls
'''

import numpy as np

def decision_step(Rover):

    if Rover.nav_angles is not None:
        if Rover.mode == 'forward':
            # Check extent of navigable terrain:
            if len(Rover.nav_angles) >= Rover.stop_forward:
                if Rover.vel < Rover.max_vel:
                    Rover.throttle = Rover.throttle_set
                else: # coast
                    Rover.throtte = 0
                Rover.brake = 0
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),-15,15)
            else:
                Rover.throtte = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode == 'stop'

        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throtte = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            else: # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15 # Could be more clever here about which way to turn
                if len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.throtte = Rover.throttle_set
                    Rover.brake = 0
                    Rover.steer = np.clip(np.mean(Rover.nav_angles)*180/np.pi, -15, 15)
                    Rover.mode = 'forward'
    else:
        # Just to make the rover do something 
        # even if no modifications have been made to the code
        Rover.throtte = 0
        Rover.steer = 15
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover
