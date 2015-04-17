'''In this exercise you have to implement a PD controller to move a point mass to a desired position by 
exerting a force. The system can be described by the following equations:

                                          xt=Δt⋅vt−1

                                          vt=Δt⋅at−1

                                            at=Fm

                                            F=ut

Where x is the position,  v the velocity,  a the acceleration,  m a fixed mass,  F the exerted force, and  u
the control command computed by the PD controller.

Implement a PD controller in the method compute_control_command in the code given below. Its input is the 
current position xt and the the desired position in meters. The method should return the control command ut. 
Tune the controller gains to achieve stability within 10s and without overshooting more than 1m.'''

class UserCode:
    def __init__(self):
        # TODO: tune gains
        self.Kp = 0.95
        self.Kd = 2.5
        self.prev_x=0
            
    def compute_control_command(self, t, dt, x_measured, x_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to compute_control_command
        :param x_measured: measured position (scalar)
        :param x_desired: desired position (scalar)
        :return - control command u
        '''
        # TODO: implement PD controller
        error = x_desired-x_measured
        error_prev = x_desired-self.prev_x
        u = self.Kp*error+(self.Kd*(error-error_prev))/dt
        self.prev_x = x_measured        
        return u

