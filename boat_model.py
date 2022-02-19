# This contains the model for the boat.
# It should be a model for the steering characteristics
# of a rowing boat.
# It is designed to allow for debugging of the steering software
# without the need for on water testing.

import math
import numpy as np

class Boat:
    def __init__(self, x, y, V, initialTheta, initialThetaDot, initialThetaDotDot, initialDelta, R, T):
        ''' Initializes a boat instance.
        '''
        self.pos = np.array([x,y]) # an array that defines the position of the
        self.V = V # Velocity of the boat
        self.theta = initialTheta # Initial heading
        self.thetaDot = initialThetaDot # Initial rate of rotation
        self.thetaDotDot = initialThetaDotDot # Initial rate of change of rate of rotation
        self.delta = initialDelta # Initial rudder angle
        self.R = R  # As used in Fossen's nomoto model
        self.T = T  # Ditto
        
    
    def step(self, delta):
        self.delta = delta
        self.pos = 
        self.theta = 
        self.thetaDot = 
        self.thetaDotDot = 
        