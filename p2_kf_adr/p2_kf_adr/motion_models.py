import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        # TODO: Define and return the 3x3 identity matrix A
        
        A = np.identity(3)
        
        return A

    def control_input_matrix_B(mu=np.zeros(3), delta_t=0.1):
        # TODO: Define B using current theta and timestep delta_t
        # B should apply linear and angular velocity to position and heading

        theta = mu[2]
        B = np.zeros((3, 2))
        B[0, 0] = np.cos(theta) * delta_t  
        B[1, 0] = np.sin(theta) * delta_t
        B[2, 1] = delta_t  

        return B

    return state_transition_matrix_A, control_input_matrix_B
def velocity_motion_model_2():
    def A(dt):
        # TODO: Define and return the 6x6 constant velocity model transition matrix
        
        A = np.eye(6)
        A[0, 3] = dt 
        A[1, 4] = dt  
        A[2, 5] = dt  

        return A

    def B(mu, dt):
        # TODO: Return 6x2 zero matrix (no control input used in pure KF)
        
        B = np.zeros([6,2])
        
        return B

    return A, B
