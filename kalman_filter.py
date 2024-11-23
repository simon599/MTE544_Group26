

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P, Q, R, x, dt):
        
        self.P = P # state covariance matrix
        self.Q = Q # process noise covariance matrix
        self.R = R # measurement noise covariance matrix
        self.x = x # state vector
        self.dt = dt # time step for predictions
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):
        # state transition matrix A, measurement matrix C (in the lecture, C is matrix H)
        self.A = self.jacobian_A()
        self.C = self.jacobian_H()
        
        # update state based on motion model
        self.motion_model()
        
        # update state covariance matrix using process model and noise
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Replace the matrices with Jacobians where needed (already completed code)
    def update(self, z):
        # compute covariance of innovation
        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R
            
        # compute Kalman gain
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        # compute innovation as difference between observed and predicted measurement
        surprise_error= z - self.measurement_model()
        
        # update state estimate using Kalman gain and innovation
        self.x=self.x + np.dot(kalman_gain, surprise_error)

        # update state covariance matrix
        self.P=np.dot((np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v, # v, linear velocity
            w, # w, rotation speed
            vdot, # ax, linear acceleration 
            v * w, # ay, rotation speed * linear velocity
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        # unpack state vector and time step
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        # update state vector using motion model equations
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        

    def jacobian_A(self): # jacobian between predict state over the previous state
        # unpack state vector and time step
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        # here, we are taking the partial derivatives of all our motion model equations
        # with respect to the state vector
        return np.array([
            #x, y,                   th, w,             v, vdot
            [1, 0, -v * np.sin(th) * dt, 0, np.cos(th) * dt,  0],
            [0, 1,  v * np.cos(th) * dt, 0, np.sin(th) * dt,  0],
            [0, 0,                    1, dt,              0,  0],
            [0, 0,                    0, 1,               0,  0],
            [0, 0,                    0, 0,               1, dt],
            [0, 0,                    0, 0,               0,  1]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        # unpack state vector
        x, y, th, w, v, vdot=self.x

        # take partial derivatives of measurement model with respect to the state vector
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        # return the current state vector
        return self.x
