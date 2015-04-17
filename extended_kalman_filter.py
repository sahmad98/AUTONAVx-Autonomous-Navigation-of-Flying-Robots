'''
In this exercise you will extend last week's Kalman filter to the nonlinear case, which is then called an Extended 
Kalman Filter (EKF). We make the following changes:

    the state includes heading (yaw) of the quadrotor,
    replace the constant velocity model with the odometry you implemented in Week 2,
    replace the GPS sensor with a camera, which detects markers in the environment. This allows us to measure the quadrotors heading as well.

In the following we describe the quadrotors state, the state and measurement prediction function.

State

The quadrotors state x includes its x and y position in world coordinates and the current heading ψ:

x=⎛⎝⎜xyψ⎞⎠⎟

State Prediction Function

The prediction function tells us what the next state xt+1 will be given our current state  xt and control 
inputs ut, i.e.,

xt+1=f(xt,ut)

Here we will use the odometry from Week 2 to predict the next state. This means we will integrate up our 
current, noisy velocity measurements. Of cause, we will accumulate drift over time, which we will correct 
with the marker observations. Using the odometry f can be defined as:

f(xt,ut)=⎛⎝⎜xt+1yt+1ψt+1⎞⎠⎟=⎛⎝⎜xt+Δt⋅(cos(ψt)⋅x˙t−sin(ψt)⋅y˙t)yt+Δt⋅(sin(ψt)⋅x˙t+cos(ψt)⋅y˙t)ψt+Δt⋅ψ
˙t⎞⎠⎟

Measurement Prediction Function

To correct for the drift we accumulate by integrating the odometry, we placed markers in the world, which 
can be detected by the down facing quadrotor camera. For each marker we know its position and orientation 
in the world. Therefore, we can predict the pose of a marker relative to the quadrotor given our current 
state xt and the world pose of the marker. By comparing the predicted relative pose and the actual measured
relative pose we can correct our state estimate. The following image illustrates the involved transformations:

The quadrotor pose TQW is given by our current state estimate, the marker pose TMW is known from our marker 
map, and the relative marker pose TMQ can be measured by the camera.

A marker measurement comprises the relative xm and ym position and the relative orientation ψm:

z=⎛⎝⎜xmymψm⎞⎠⎟

The measurement prediction function z=h(xt) is then defined as

h(xt)=⎛⎝⎜⎜cos(ψt)⋅(xMW−xt)+sin(ψt)⋅(yMW−yt)−sin(ψt)⋅(xMW−xt)+cos(ψt)⋅(yMW−yt)ψMW−ψt⎞⎠⎟⎟

where xMW,yMW, and ψMW are the world marker position and orientation.



For the implementation of the EKF we will need the Jacobian matrices of the state prediction function f 
and the measurement prediction function h. The 3×3 Jacobian matrix of f is defined as:

F=∂f(x,u)∂x∣∣∣xt=⎛⎝⎜100010Δt⋅(−sin(ψt)⋅x˙t−cos(ψt)⋅y˙t)Δt⋅(cos(ψt)⋅x˙t−sin(ψt)⋅y˙t)1⎞⎠⎟

The Jacobian matrix of h is as well a 3×3 matrix. It is defined as:

H=∂h(x)∂x∣∣∣xt=⎛⎝⎜H1,1H2,10H1,2H2,20H1,3H2,3−1⎞⎠⎟

Please calculate the missing 6 entries, i.e., H1,1=∂xm∂xt, H1,2=∂xm∂yt, H1,3=∂xm∂ψt, H2,1=∂ym∂xt, H2,2=∂ym∂yt, 
and H2,3=∂ym∂ψt.

You can use sin, cos functions, * for multiplication, x and y for the drone position, psi for the drone yaw 
angle, x_m and y_m for the marker world position. 

The code below provides a framework for the EKF described above. It is an adapted version of last week. The 
goal is again to fly the quadrotor along a square trajectory. This time the heading is also controlled.

The prediction steps have already been implemented. For the correction step the method 
calculatePredictMeasurementJacobian is missing, which computes H given the current state and a global marker 
pose. Use your solutions from above. Furthermore, you have to implement the correctState method, which performs
the EKF state correction step.

If your solution is correct you will get a similar output to the image shown below. In the simulator, the 
pink trajectory visualizes the Kalman filter state. The orange lines indicate that the quadrotor observes 
a marker. Note that the quadrotor can only observe a marker while it is located above the marker. The blue 
line is the groundtruth trajectory. The cyan line shows the setpoint position.
'''

import math
import numpy as np
from plot import plot_trajectory, plot_point, plot_covariance_2d
        
class UserCode:
    def __init__(self):        
        #TODO: Play with the noise matrices
        
        #process noise
        pos_noise_std = 0.005
        yaw_noise_std = 0.005
        self.Q = np.array([
            [pos_noise_std*pos_noise_std,0,0],
            [0,pos_noise_std*pos_noise_std,0],
            [0,0,yaw_noise_std*yaw_noise_std]
        ]) 
        
        #measurement noise
        z_pos_noise_std = 0.005
        z_yaw_noise_std = 0.03
        self.R = np.array([
            [z_pos_noise_std*z_pos_noise_std,0,0],
            [0,z_pos_noise_std*z_pos_noise_std,0],
            [0,0,z_yaw_noise_std*z_yaw_noise_std]
        ])
        
        # state vector [x, y, yaw] in world coordinates
        self.x = np.zeros((3,1)) 
        
        # 3x3 state covariance matrix
        self.sigma = 0.01 * np.identity(3) 
        
    def rotation(self, yaw):
        '''
        create 2D rotation matrix from given angle
        '''
        s_yaw = math.sin(yaw)
        c_yaw = math.cos(yaw)
                
        return np.array([
            [c_yaw, -s_yaw], 
            [s_yaw,  c_yaw]
        ])
    
    def normalizeYaw(self, y):
        '''
        normalizes the given angle to the interval [-pi, +pi]
        '''
        while(y > math.pi):
            y -= 2 * math.pi
        while(y < -math.pi):
            y += 2 * math.pi
        return y
    
    def visualizeState(self):
        # visualize position state
        plot_trajectory("kalman", self.x[0:2])
        plot_covariance_2d("kalman", self.sigma[0:2,0:2])
    
    def predictState(self, dt, x, u_linear_velocity, u_yaw_velocity):
        '''
        predicts the next state using the current state and 
        the control inputs local linear velocity and yaw velocity
        '''
        x_p = np.zeros((3, 1))
        x_p[0:2] = x[0:2] + dt * np.dot(self.rotation(x[2]), u_linear_velocity)
        x_p[2]   = x[2]   + dt * u_yaw_velocity
        x_p[2]   = self.normalizeYaw(x_p[2])
        
        return x_p
    
    def calculatePredictStateJacobian(self, dt, x, u_linear_velocity, u_yaw_velocity):
        '''
        calculates the 3x3 Jacobian matrix for the predictState(...) function
        '''
        s_yaw = math.sin(x[2])
        c_yaw = math.cos(x[2])
        
        dRotation_dYaw = np.array([
            [-s_yaw, -c_yaw],
            [ c_yaw, -s_yaw]
        ])
        F = np.identity(3)
        F[0:2, 2] = dt * np.dot(dRotation_dYaw, u_linear_velocity)
        
        return F
    
    def predictCovariance(self, sigma, F, Q):
        '''
        predicts the next state covariance given the current covariance, 
        the Jacobian of the predictState(...) function F and the process noise Q
        '''
        return np.dot(F, np.dot(sigma, F.T)) + Q
    
    def calculateKalmanGain(self, sigma_p, H, R):
        '''
        calculates the Kalman gain
        '''
        return np.dot(np.dot(sigma_p, H.T), np.linalg.inv(np.dot(H, np.dot(sigma_p, H.T)) + R))
    
    def correctState(self, K, x_predicted, z, z_predicted):
        '''
        corrects the current state prediction using Kalman gain, the measurement and the predicted measurement
        
        :param K - Kalman gain
        :param x_predicted - predicted state 3x1 vector
        :param z - measurement 3x1 vector
        :param z_predicted - predicted measurement 3x1 vector
        :return corrected state as 3x1 vector
        '''
        
        # TODO: implement correction of predicted state x_predicted
        
        #z = self.normalizeYaw(z)
        #z_predicted = self.normalizeYaw(z_predicted)
        x_predicted = x_predicted + np.dot(K,(z-z_predicted))
        x_predicted[2] = self.normalizeYaw(x_predicted[2])
        return x_predicted
    
    def correctCovariance(self, sigma_p, K, H):
        '''
        corrects the sate covariance matrix using Kalman gain and the Jacobian matrix of the predictMeasurement(...) function
        '''
        return np.dot(np.identity(3) - np.dot(K, H), sigma_p)
    
    def predictMeasurement(self, x, marker_position_world, marker_yaw_world):
        '''
        predicts a marker measurement given the current state and the marker position and orientation in world coordinates 
        '''
        z_predicted = Pose2D(self.rotation(x[2]), x[0:2]).inv() * Pose2D(self.rotation(marker_yaw_world), marker_position_world);
        
        return np.array([[z_predicted.translation[0], z_predicted.translation[1], z_predicted.yaw()]]).T
    
    def calculatePredictMeasurementJacobian(self, x, marker_position_world, marker_yaw_world):
        '''
        calculates the 3x3 Jacobian matrix of the predictMeasurement(...) function using the current state and 
        the marker position and orientation in world coordinates
        
        :param x - current state 3x1 vector
        :param marker_position_world - x and y position of the marker in world coordinates 2x1 vector
        :param marker_yaw_world - orientation of the marker in world coordinates
        :return - 3x3 Jacobian matrix of the predictMeasurement(...) function
        '''
        
        # TODO: implement computation of H
        s_yaw = math.sin(x[2])
        c_yaw = math.cos(x[2])
        dx = marker_position_world[0]-x[0]
        dy = marker_position_world[1]-x[1]
        H = np.array([[-c_yaw,-s_yaw,((-s_yaw*dx)+(c_yaw*dy))],[s_yaw,-c_yaw,((-c_yaw*dx)+(-s_yaw*dy))],[0,0,-1]])
        #print H
        return H
    
    def state_callback(self, t, dt, linear_velocity, yaw_velocity):
        '''
        called when a new odometry measurement arrives approx. 200Hz
        
        :param t - simulation time
        :param dt - time difference this last invocation
        :param linear_velocity - x and y velocity in local quadrotor coordinate frame (independet of roll and pitch)
        :param yaw_velocity - velocity around quadrotor z axis (independet of roll and pitch)
        '''
        self.x = self.predictState(dt, self.x, linear_velocity, yaw_velocity)
        
        F = self.calculatePredictStateJacobian(dt, self.x, linear_velocity, yaw_velocity)
        self.sigma = self.predictCovariance(self.sigma, F, self.Q);
        
        self.visualizeState()
    
    def measurement_callback(self, marker_position_world, marker_yaw_world, marker_position_relative, marker_yaw_relative):
        '''
        called when a new marker measurement arrives max 30Hz, marker measurements are only available if the quadrotor is
        sufficiently close to a marker
        
        :param marker_position_world - x and y position of the marker in world coordinates 2x1 vector
        :param marker_yaw_world - orientation of the marker in world coordinates
        :param marker_position_relative - x and y position of the marker relative to the quadrotor 2x1 vector
        :param marker_yaw_relative - orientation of the marker relative to the quadrotor
        '''
        z = np.array([[marker_position_relative[0], marker_position_relative[1], marker_yaw_relative]]).T
        z_predicted = self.predictMeasurement(self.x, marker_position_world, marker_yaw_world)
                
        H = self.calculatePredictMeasurementJacobian(self.x, marker_position_world, marker_yaw_world)
        K = self.calculateKalmanGain(self.sigma, H, self.R)
        
        self.x = self.correctState(K, self.x, z, z_predicted)
        self.sigma = self.correctCovariance(self.sigma, K, H)
        
        self.visualizeState()
    
class Pose2D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation
        
    def inv(self):
        '''
        inversion of this Pose2D object
        
        :return - inverse of self
        '''
        inv_rotation = self.rotation.transpose()
        inv_translation = -np.dot(inv_rotation, self.translation)
        
        return Pose2D(inv_rotation, inv_translation)
    
    def yaw(self):
        from math import atan2
        return atan2(self.rotation[1,0], self.rotation[0,0])
        
    def __mul__(self, other):
        '''
        multiplication of two Pose2D objects, e.g.:
            a = Pose2D(...) # = self
            b = Pose2D(...) # = other
            c = a * b       # = return value
        
        :param other - Pose2D right hand side
        :return - product of self and other
        '''
        return Pose2D(np.dot(self.rotation, other.rotation), np.dot(self.rotation, other.translation) + self.translation)



