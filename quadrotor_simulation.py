'''In this exercise we want to familiarize you with the interface between the simulation and your code. Furthermore,
we show you how to plot data over time.

The quadrotor uses the coordinate system convention depicted in the following image. The X axis points 
forward, the Y axis to the left, and the Z axis upwards.

Code Interface

The main interface between the simulation and your code is the method measurement_callback defined in the class 
UserCode. The method has three parameters the current simulation time t, the time difference since the last invocation 
dt, and an object containing various measurements of the quadrotor called navdata. The data stored in the navdata 
object is explained below.

Plotting

To plot data you can use the plot function. The first argument is the name of the value you want to plot. 
The second argument is the actual value. This value will be internally associated with the current simulation 
time. All the graphs are displayed in the area above the code editor.

Navdata

The navdata contains the following values:

    rotX - roll angle in radians (rotation around x axis)
    rotY - pitch angle in radians (rotation around y axis)
    rotZ - yaw angle in radians (rotation around z axis)
    vx - velocity along the x axis
    vy - velocity along the y axis

Below the quadrotor simulation are a couple of questions, which you can answer by plotting the different 
measurements. Fly the quadrotor manually using the keyboard control.
'''

from plot import plot

class UserCode:
    def __init__(self):
        # initialize data you want to store in this object between calls to the measurement_callback() method
        self.last_yaw = 0
        
    def measurement_callback(self, t, dt, navdata):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        # add your plot commands here
        plot("roll", navdata.rotX);
