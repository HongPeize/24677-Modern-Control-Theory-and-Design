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
        pid_input_max=1
        pid_input_min=0
        # Add additional member variables according to your need here.
        self.cumulative_error = 0
        self.previous_error = 0
        #self.pid_input_max=max(pid_input)
        #self.pid_input_min=min(pid_input)
        
    def cal_PID(self, current_error, Kp, Ki, Kd):
        delt_T=0.01
        differential_error = current_error - (self.previous_error if (self.previous_error is not None) else current_error)
        
        
        self.cumulative_error+=current_error * delt_T
        self.previous_error = current_error
        pid_input = Kp * current_error + Ki * self.cumulative_error + Kd * differential_error
        #self.pid_input=clamp(self.pid_input, self.pid_input_min, self.pid_input_max) #avoid blowup

        return pid_input
        
    def update(self, timestep):

        delt_T=0.01
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

        time_horizon=10 #"looks ahead" mechanism
        _, closest_index = closestNode(X,Y,trajectory)
        X_desired = trajectory[closest_index + time_horizon,0]
        Y_desired = trajectory[closest_index + time_horizon,1]
        psi_desired = np.arctan2(Y_desired - Y, X_desired - X)
            
         #---------------|Lateral Controller|-------------------------
        
        #Please design your lateral controller below.
        psi_error = wrapToPi(psi_desired - psi)
        delta = self.cal_PID(psi_error, 5, 0., 0.)
        delta=clamp(delta, -np.pi/6, np.pi/6) #to comply assignment requirement

        """

        # ---------------|Longitudinal Controller|-------------------------
        """
        #Please design your longitudinal controller below.
        
        position_error = np.power(np.power(X_desired - X, 2) + np.power(Y_desired - Y, 2), 0.5)
        velocity_error = position_error / delt_T
        
        F = self.cal_PID(velocity_error, 5, 0., 0.)
        F=clamp(F, 0.01, 15736) #to comply assignment requirement
        
        # Return all states and calculated control inputs (F, delta)
        return X_desired, Y_desired, xdot, ydot, psi_desired, psidot, F, delta
        
    #getTrajectory(your_controller)
