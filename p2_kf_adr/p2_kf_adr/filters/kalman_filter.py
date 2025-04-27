import numpy as np 

from ..motion_models import velocity_motion_model, velocity_motion_model_2
from ..observation_models import odometry_observation_model, odometry_observation_model_2

class KalmanFilter:

    def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.02, 0.02, 0.01], obs_noise_std = [0.02, 0.02, 0.01]):
        self.mu = initial_state # Initial state estimate [x, y, theta]
        self.sigma = initial_covariance # Initial uncertainty

        self.A, self.B = velocity_motion_model() # The action model to use. Returns A and B matrices

        # Standard deviations for the noise in x, y, and theta (process or action model noise)
        self.proc_noise_std = np.array(proc_noise_std)
        # Process noise covariance (R)
        self.R = np.diag(self.proc_noise_std ** 2)  # process noise covariance

        # Observation model (C)
        self.C = odometry_observation_model() # The observation model to use

        # Standard deviations for the noise in x, y, theta (observation or sensor model noise)
        self.obs_noise_std = np.array(obs_noise_std)
        # Observation noise covariance (Q)
        self.Q = np.diag(self.obs_noise_std ** 2)
            
    def predict(self, u, dt):
        # TODO: Implement Kalman filter prediction step
        A = self.A()
        B = self.B(self.mu, dt)

        # Predict the new mean (mu) using A, B, and control input u
        self.mup = np.dot(A , self.mu) + np.dot(B , u)
        
        # Predict the new covariance (Sigma) using A and R
        self.sigmap = np.dot(np.dot(A , self.sigma) , np.transpose(A)) + self.R
        
        pass

    def update(self, z):
        # TODO: Implement Kalman filter correction step
       
        # Compute Kalman gain K
        K = np.dot(np.dot(self.sigmap , np.transpose(self.C)) , np.linalg.inv(np.dot(np.dot(self.C , self.sigmap) , np.transpose(self.C)) + self.Q))
        
        # Update the mean (mu) with the measurement z
        self.mu = self.mup + np.dot(K , z - np.dot(self.C , self.mup)) 
        
        # Update the covariance (Sigma)
        self.sigma= np.dot(np.identity(3) - np.dot(K , self.C) , self.sigmap)
        
        return (self.mu , self.sigma)

class KalmanFilter_2:
    def __init__(self, initial_state, initial_covariance,
                 proc_noise_std=[0.02]*6, obs_noise_std=[0.02]*6):

        self.mu = initial_state  # Initial state estimate [x, y, theta, vx, vy, omega]
        self.sigma = initial_covariance  # Initial uncertainty

        self.A, self.B = velocity_motion_model_2()  # Motion model matrices

        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # Process noise covariance

        self.C = odometry_observation_model_2()  # Observation matrix
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Observation noise covariance

    def predict(self, u=None, dt=1.0):
        # TODO: Implement Kalman prediction step for full state (6D)
        A = self.A(dt)
        B = self.B(self.mu, dt)
        
        # Pure KF: use only the A matrix to update the state and covariance
        self.mup = np.dot(A , self.mu) + np.dot(B , u)
        
        self.sigmap = np.dot(np.dot(A , self.sigma) , np.transpose(A)) + self.R
        
        pass

    def update(self, z):
        # TODO: Implement update step
        
        # Compute Kalman gain
        K = np.dot(np.dot(self.sigmap , np.transpose(self.C)) , np.linalg.inv(np.dot(np.dot(self.C , self.sigmap) , np.transpose(self.C)) + self.Q))
        
        # Correct the predicted state with measurement
        self.mu = self.mup + np.dot(K , z - np.dot(self.C , self.mup))
        
        # Update covariance
        self.sigma= np.dot(np.identity(6) - np.dot(K , self.C) , self.sigmap)
        
        return (self.mu , self.sigma)
