# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *

# CustomController class (inherits from BaseController)
class CustomController(BaseController):

    def __init__(self, trajectory):

        super().__init__(trajectory)

        # Define constants
        # These can be ignored in P1
        self.lr = 1.39
        self.lf = 1.55
        self.Ca = 20000
        self.Iz = 25854
        self.m = 1888.6
        self.g = 9.81
        # Add additional member variables according to your need here.
        self.cumulative_error = 0
        self.previous_error = 0
       
        
    def cal_PID(self, delt_T, current_error, Kp, Ki, Kd):
        #delt_T=0.05
        differential_error = (current_error - self.previous_error)/delt_T
        self.cumulative_error+=current_error * delt_T
        self.previous_error = current_error
        pid_input = Kp * current_error + Ki * self.cumulative_error + Kd * differential_error
        #self.pid_input=clamp(self.pid_input, self.pid_input_min, self.pid_input_max) #avoid blowup

        return pid_input
        
    def update(self, timestep):

        trajectory = self.trajectory

        lr = self.lr
        lf = self.lf
        Ca = self.Ca
        Iz = self.Iz
        m = self.m
        g = self.g

        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)

        # Design your controllers in the spaces below.
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta)

        forwardindex=20 #"looks ahead" mechanism
        _, closest_index = closestNode(X,Y,trajectory)
        if forwardindex + closest_index >= 8203:
           forwardindex=0

        X_desired = trajectory[closest_index + forwardindex,0]
        Y_desired = trajectory[closest_index + forwardindex,1]
        
        psi_desired = np.arctan2(Y_desired - Y, X_desired - X)
        x_velocity=30 #desired velocity
                    
         #---------------|Lateral Controller|-------------------------
        
        #Please design your lateral controller below.
        A = np.array([[0, 1, 0, 0], [0, -4*Ca / (m * xdot), 4*Ca / m, -(2*Ca*(lf - lr))/(m*xdot)], [0, 0, 0, 1], [0, -(2*Ca*(lf - lr)) / (Iz * xdot), (2*Ca*(lf - lr)) / Iz, (-2*Ca*(np.power(lf, 2) + np.power(lr, 2))) / (Iz * xdot)]])
        B = np.array([[0], [2*Ca / m], [0], [(2 * Ca* lf) / Iz]])
        P = np.array([-20, -10, -1, 0])
        K = signal.place_poles(A, B, P)
        K=K.gain_matrix
        e1 = 0
        e2 = wrapToPi(psi - psi_desired)
        e1dot = ydot + xdot * e2
        e2dot = psidot

        e = np.hstack((e1, e1dot, e2, e2dot))

        delta = -np.matmul(K,e)
        delta=float(delta)
        #psi_error = wrapToPi(psi_desired - psi)
        #delta = self.cal_PID(psi_error, 5, 0., 0.)
        #delta=clamp(delta, -np.pi/6, np.pi/6) #to comply assignment requirement

        """

        # ---------------|Longitudinal Controller|-------------------------
        """
        #Please design your longitudinal controller below.
        
        #position_error = np.power(np.power(X_desired - X, 2) + np.power(Y_desired - Y, 2), 0.5)
        velocity_error = x_velocity-xdot
        F = self.cal_PID(delT, velocity_error, 50, 0.0001, 0.0001)
        #F=clamp(F, 0.01, 15736) #to comply assignment requirement
        
        # Return all states and calculated control inputs (F, delta)
        return X_desired, Y_desired, xdot, ydot, psi_desired, psidot, F, delta
        
    #getTrajectory(your_controller)
