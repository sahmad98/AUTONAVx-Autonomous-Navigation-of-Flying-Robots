'''The goal of this exercise is to compute the current position of the quadrotor by integrating its velocity 
measurements over time.

Use the plot_trajectory function to visualize the estimated position. The first parameter is the name of 
the trajectory. The second parameter is a 2 or 3 dimensional column vector containing the current x, y and 
optionally z position. Your result should look similar to the image below.

Tip: if you need trigonometric functions use from math import sin, cos
'''

import numpy as np
from plot import plot_trajectory
from math import sin, cos
class UserCode:
    def __init__(self):
        self.position = np.array([[0], [0]])
    
    def measurement_callback(self, t, dt, navdata):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        
        # TODO: update self.position by integrating measurements contained in navdata
        plot_trajectory("odometry", self.position)
        distance = navdata.vx*dt
        dx = distance*cos(navdata.rotZ)
        dy = distance*sin(navdata.rotZ)
        self.position = self.position + np.array([[dx],[dy]])
