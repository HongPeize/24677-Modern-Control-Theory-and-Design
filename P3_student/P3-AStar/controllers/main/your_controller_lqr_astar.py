# Fill in the respective functions to implement the LQR optimal controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *
from scipy.signal import StateSpace, lsim, dlsim


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
        self.cumulative_error = 0
        self.previous_error = 0
        
    def cal_PID(self, delt_T, current_error, Kp, Ki, Kd):
        differential_error = (current_error - self.previous_error)/delt_T
        self.cumulative_error+=current_error * delt_T
        self.previous_error = current_error
        pid_input = Kp * current_error + Ki * self.cumulative_error + Kd * differential_error
        return pid_input
        
    def dlqr(self,A,B,Q,R):
        S = np.matrix(linalg.solve_discrete_are(A, B, Q, R))
        K = -np.matrix(linalg.inv(B.T@S@B+R)@(B.T@S@A))
        return K
        
    def update(self, timestep):

        trajectory = self.trajectory

        lr = self.lr
        lf = self.lf
        Ca = self.Ca
        Iz = self.Iz
        m = self.m
        g = self.g

        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot, obstacleX, obstacleY = super().getStates(timestep)
        
        forwardindex=80 #"looks ahead" mechanism
        _, closest_index = closestNode(X,Y,trajectory)
        if forwardindex + closest_index >= 6596:
           forwardindex=0

        X_desired = trajectory[closest_index + forwardindex,0]
        Y_desired = trajectory[closest_index + forwardindex,1]
        
        psi_desired = np.arctan2(Y_desired - Y, X_desired - X)
        x_velocity=15 #desired velocity
        
        # ---------------|Lateral Controller|-------------------------
       
        A = np.array([[0, 1, 0, 0], [0, -4*Ca / (m * xdot), 4*Ca / m, -(2*Ca*(lf - lr))/(m*xdot)], [0, 0, 0, 1], [0, -(2*Ca*(lf - lr)) / (Iz * xdot), (2*Ca*(lf - lr)) / Iz, (-2*Ca*(np.power(lf, 2) + np.power(lr, 2))) / (Iz * xdot)]])
        B = np.array([[0], [2*Ca / m], [0], [(2 * Ca* lf) / Iz]]) 
        #C=  np.array([[0], [1], [0], [0]]) 
        C = np.identity(4)
        D = np.array([[0],[0],[0],[0]])
        sys_ct = StateSpace(A, B, C, D)
        sys_dt= sys_ct.to_discrete(delT)
        dt_A = sys_dt.A
        dt_B = sys_dt.B
        #Q=np.array([[3,0,0,0], [0,0.0011,0,0], [0,0,0.06,0], [0,0,0,0.00345]])
        R=10
        Q=np.array([[1000,0,0,0], [0,100,0,0], [0,0,60,0], [0,0,0,90]])
        #Q=np.eye(4)
        #R=10
        K= self.dlqr(dt_A,dt_B,Q,R)
        
        e1 = 0
        e2 = wrapToPi(psi - psi_desired)
        e1dot = ydot + xdot * e2
        e2dot = psidot
        e = np.hstack((e1, e1dot, e2, e2dot))
        delta = float(np.matmul(K,e))
        #delta=clamp(delta, -np.pi/6, np.pi/6) #to comply assignment requirement

        # ---------------|Longitudinal Controller|-------------------------
      
        velocity_error = x_velocity-xdot
        F = self.cal_PID(delT, velocity_error, 200, 0.0001, 0.0001)
        #F=clamp(F, 0.01, 15736) #to comply assignment requirement
        # Return all states and calculated control inputs (F, delta) and obstacle position
        return X, Y, xdot, ydot, psi, psidot, F, delta, obstacleX, obstacleY
