

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        # P: Initial estimate covariance matrix (6x6)
        # Q: Process noise covariance matrix (6x6)
        # R: Measurement noise covariance matrix (4x4)
        # x: Initial state vector [x, y, th, w, v, vdot]^T
        # dt: Time step
        self.P = P  # Estimate covariance matrix
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.x = x  # State vector
        self.dt = dt  # Time step
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        self.A = self.jacobian_A()
        self.C = self.jacobian_A()
        
        self.motion_model()
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):
        # Compute the Jacobian of the measurement model (C matrix)
        self.C = self.jacobian_H()
        
        # Compute the innovation covariance (S matrix)
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        
        # Compute the Kalman Gain (K matrix)
        kalman_gain = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        # Compute the measurement residual (innovation)
        surprise_error = z - self.measurement_model()
        
        # Update the state estimate
        self.x = self.x + np.dot(kalman_gain, surprise_error)
        
        # Update the estimate covariance matrix
        self.P = np.dot((np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)), self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        # Compute the expected measurements based on the current state
        # Measurements: linear velocity v, angular velocity w, linear accelerations ax and ay
        return np.array([
            v,           # Linear velocity
            w,           # Angular velocity
            vdot,        # Linear acceleration in x-direction (robot frame)
            v * w        # Linear acceleration in y-direction (robot frame)
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        # Update the state based on the motion model equations
        self.x = np.array([
            x + v * np.cos(th) * dt,  # Update x position
            y + v * np.sin(th) * dt,  # Update y position
            th + w * dt,              # Update orientation
            w,                        # Angular velocity remains the same
            v + vdot * dt,            # Update linear velocity
            vdot                      # Linear acceleration remains the same
        ])
        
    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        # Compute the partial derivatives for the Jacobian A
        return np.array([
            #      x, y,               th,                   w,             v, vdot
            [1,    0, -v * np.sin(th) * dt, 0, np.cos(th) * dt,  0],  # ∂f/∂x
            [0,    1,  v * np.cos(th) * dt, 0, np.sin(th) * dt,  0],  # ∂f/∂y
            [0,    0,              1,        dt,           0,     0],  # ∂f/∂th
            [0,    0,              0,        1,            0,     0],  # ∂f/∂w
            [0,    0,              0,        0,            1,     dt], # ∂f/∂v
            [0,    0,              0,        0,            0,     1 ]  # ∂f/∂vdot
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot = self.x
        # Compute the partial derivatives for the Jacobian C
        return np.array([
            # x, y, th, w, v, vdot
            [0, 0, 0,  0, 1,     0],    # ∂h1/∂x (v)
            [0, 0, 0,  1, 0,     0],    # ∂h2/∂x (w)
            [0, 0, 0,  0, 0,     1],    # ∂h3/∂x (vdot)
            [0, 0, 0,  v, w,     0]     # ∂h4/∂x (v * w)
        ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        return self.x
