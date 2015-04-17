'''In this exercise you will implement your own Kalman filter to correct noisy sensor measurements. The output of 
the Kalman filter will be fed into a PD controller to guide the quadrotor along a square trajectory.Your task is to
implement the prediction and correction step of the Kalman filter.

The code below implements a linear Kalman filter. The state_callback method is invoked in every simulation step 
and performs one Kalman filter prediction step (200Hz). The measurement_callback method is called when a new GPS 
sensor reading arrives and calculates the Kalman filter correction step (13Hz). The GPS reading gives a noisy
measurement of the absolute position in world coordinates.

You have to implement the predictState and the correctState method. The covariance prediction, correction and
the Kalman gain calculation are already implemented.

The noisy GPS measurements are visualized with orange flares. Initially, the quadrotor is located at the world 
origin. From these measurements, your task is to estimate the pose of the quadrotor using the Kalman filter. The 
estimated trajectory is visualized in pink. This estimated pose is then fed into a PD controller to generate the 
control commands to move the quadrotor along a square trajectory. The actual trajectory of the quadrotor is then 
visualized with the blue line. The goal of this exercise is to finish the Kalman filter such that the estimated 
trajectory (pink) matches the true trajectory (blue) as good as possible. Note that the match will not be 
perfect, because of noise of the measurements and time delays of the estimation. The grader will check whether
the estimated trajectory is sufficiently close to the true trajectory.

The PD controller will access the self.x member variable to get the current state! Please do not modify the
measurement_callback and state_callback method.'''

import numpy as np
from plot import plot_trajectory, plot_point, plot_covariance_2d

class UserCode:
    def __init__(self):
        dt = 0.005
        
        #State-transition model
        self.A = np.array([
            [1,0,dt,0],
            [0,1,0,dt],
            [0,0,1,0],
            [0,0,0,1]
        ]) 
        #Observation model
        self.H = np.array([[1,0,0,0],[0,1,0,0]]) 
        
        #TODO: Play with the noise matrices
        #Process/State noise
        vel_noise_std = 0.005
        pos_noise_std = 0.005
        self.Q = np.array([
            [pos_noise_std*pos_noise_std,0,0,0],
            [0,pos_noise_std*pos_noise_std,0,0],
            [0,0,vel_noise_std*vel_noise_std,0],
            [0,0,0,vel_noise_std*vel_noise_std]
        ]) 
        
        #Sensor/Measurement noise
        measurement_noise_std = 0.5
        self.R = measurement_noise_std * measurement_noise_std * np.identity(2) 

        self.x = np.zeros((4,1)) #Initial state vector [x,y,vx,vy]
        self.sigma = np.identity(4) #Initial covariance matrix
    
    def predictState(self, A, x):
        '''
        :param A: State-transition model matrix
        :param x: Current state vector
        :return x_p: Predicted state vector as 4x1 numpy array
        '''
        
        #TODO: Predict the next state
        x_p = np.zeros((4,1))
        
        x_p = np.dot(A,x) 
        
        return x_p
    
    def predictCovariance(self, A, sigma, Q):
        sigma_p = np.dot(np.dot(A, sigma), np.transpose(A))+Q
        return sigma_p
    
    def calculateKalmanGain(self, sigma_p, H, R):
        k = np.dot(np.dot(sigma_p, np.transpose(H)), np.linalg.inv(np.dot(H, np.dot(sigma_p, np.transpose(H)))+R))
        return k
    
    def correctState(self, z, x_p, k, H):
        '''
        :param z: Measurement vector
        :param x_p: Predicted state vector
        :param k: Kalman gain
        :param H: Observation model
        :return x: Corrected state vector as 4x1 numpy array
        '''
        
        #TODO: Correct the current state prediction with the measurement
        x = np.zeros((4,1))
        
        x = x_p + np.dot(k,(z-np.dot(H,x_p)))

        return x
    
    def correctCovariance(self, sigma_p, k, H):
        sigma = np.dot((np.identity(4)-np.dot(k, H)), sigma_p)
        return sigma
    
    def state_callback(self):
        self.x = self.predictState(self.A, self.x)
        self.sigma = self.predictCovariance(self.A, self.sigma, self.Q)
        
        # visualize position state
        plot_trajectory("kalman", self.x[0:2])
        plot_covariance_2d("kalman", self.sigma[0:2,0:2])
        
    def measurement_callback(self, measurement):
        '''
        :param measurement: vector of measured coordinates
        '''
        
        # visualize measurement
        plot_point("gps", measurement)
        
        k = self.calculateKalmanGain(self.sigma, self.H, self.R)
        
        self.x = self.correctState(measurement, self.x, k, self.H)
        self.sigma = self.correctCovariance(self.sigma, k, self.H)
        
        # visualize position state
        plot_trajectory("kalman", self.x[0:2])
        plot_covariance_2d("kalman", self.sigma[0:2,0:2])
