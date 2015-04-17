'''In this exercise you have to implement the 3D position control of the quadrotor. The goal is to guide the 
quadrotor along a circle trajectory based on noisy and delayed measurements.

Implement the method compute_control_command in the code given below. Its input is the current state of the 
quadrotor (state) and the desired state (state_desired). Both states are instances of the State class, which 
comprises the position and the velocity of the quadrotor. The ouput of the method are three linear velocities
commands for the X, Y, and Z axis respectively. Furthermore, you have to tune the controller gains.

They allow you to enable noise and delay for the measurements. Additionally, you can choose three different 
setpoints (desired states) for the quadrotor:

    fix - a fixed point in space (always the same)
    random - a random point in space
    circle - the setpoint changes over time along a circular trajectory

The simulation will automatically stop after 30 seconds.

Grading: Run the simulation with noise and delay enabled on the circle trajectory for at least 25 seconds of 
simulation time. Afterwards click the check button. The quadrotor has to follow the circle trajectory closely. 
If the error is too large you will not get credits! Therefore, tune your gains and try before submitting! The 
following image shows a valid solution.'''

import numpy as np

class State:
    def __init__(self):
        self.position = np.zeros((3,1))
        self.velocity = np.zeros((3,1))

class UserCode:
    def __init__(self):
        # TODO: tune gains
    
        # xy control gains
        Kp_xy = 0.5 # xy proportional
        Kd_xy = 0.0 # xy differential
        
        # height control gains
        Kp_z  = 0.5 # z proportional
        Kd_z  = 0.0 # z differential
        
        self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T
    
    def compute_control_command(self, t, dt, state, state_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param state: State - current quadrotor position and velocity computed from noisy measurements
        :param state_desired: State - desired quadrotor position and velocity
        :return - xyz velocity control signal represented as 3x1 numpy array
        '''
        # plot current state and desired setpoint
        self.plot(state.position, state_desired.position)
        
        # TODO: implement PID controller computing u from state and state_desired
        u = np.zeros((3,1))
        
        return u
        
    def plot(self, position, position_desired):
        from plot import plot
        plot("x", position[0])
        plot("x_des", position_desired[0])
        plot("y", position[1])
        plot("y_des", position_desired[1])
        plot("z", position[2])
        plot("z_des", position_desired[2])
        
