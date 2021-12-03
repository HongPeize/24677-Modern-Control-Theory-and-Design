# Fill in the respective function to implement the LQR/EKF SLAM controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from scipy.spatial.transform import Rotation
from util import *
from ekf_slam import EKF_SLAM
from scipy.signal import StateSpace, lsim, dlsim

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
        
        self.counter = 0
        np.random.seed(99)

        # Add additional member variables according to your need here.
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


    def getStates(self, timestep, use_slam=False):

        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)

        # Initialize the EKF SLAM estimation
        if self.counter == 0:
            # Load the map
            minX, maxX, minY, maxY = -120., 450., -500., 50.
            map_x = np.linspace(minX, maxX, 7)
            map_y = np.linspace(minY, maxY, 7)
            map_X, map_Y = np.meshgrid(map_x, map_y)
            map_X = map_X.reshape(-1,1)
            map_Y = map_Y.reshape(-1,1)
            self.map = np.hstack((map_X, map_Y)).reshape((-1))
            
            # Parameters for EKF SLAM
            self.n = int(len(self.map)/2)             
            X_est = X + 0.5
            Y_est = Y - 0.5
            psi_est = psi - 0.02
            mu_est = np.zeros(3+2*self.n)
            mu_est[0:3] = np.array([X_est, Y_est, psi_est])
            mu_est[3:] = np.array(self.map)
            init_P = 1*np.eye(3+2*self.n)
            W = np.zeros((3+2*self.n, 3+2*self.n))
            W[0:3, 0:3] = delT**2 * 0.1 * np.eye(3)
            V = 0.1*np.eye(2*self.n)
            V[self.n:, self.n:] = 0.01*np.eye(self.n)
            # V[self.n:] = 0.01
            print(V)
            
            # Create a SLAM
            self.slam = EKF_SLAM(mu_est, init_P, delT, W, V, self.n)
            self.counter += 1
        else:
            mu = np.zeros(3+2*self.n)
            mu[0:3] = np.array([X, 
                                Y, 
                                psi])
            mu[3:] = self.map
            y = self._compute_measurements(X, Y, psi)
            mu_est, _ = self.slam.predict_and_correct(y, self.previous_u)

        self.previous_u = np.array([xdot, ydot, psidot])

        print("True      X, Y, psi:", X, Y, psi)
        print("Estimated X, Y, psi:", mu_est[0], mu_est[1], mu_est[2])
        print("-------------------------------------------------------")
        
        if use_slam == True:
            return delT, mu_est[0], mu_est[1], xdot, ydot, mu_est[2], psidot
        else:
            return delT, X, Y, xdot, ydot, psi, psidot

    def _compute_measurements(self, X, Y, psi):
        x = np.zeros(3+2*self.n)
        x[0:3] = np.array([X, Y, psi])
        x[3:] = self.map
        
        p = x[0:2]
        psi = x[2]
        m = x[3:].reshape((-1,2))

        y = np.zeros(2*self.n)

        for i in range(self.n):
            y[i] = np.linalg.norm(m[i, :] - p)
            y[self.n+i] = wrapToPi(np.arctan2(m[i,1]-p[1], m[i,0]-p[0]) - psi)
            
        y = y + np.random.multivariate_normal(np.zeros(2*self.n), self.slam.V)
        # print(np.random.multivariate_normal(np.zeros(2*self.n), self.slam.V))
        return y

    def update(self, timestep):

        trajectory = self.trajectory

        lr = self.lr
        lf = self.lf
        Ca = self.Ca
        Iz = self.Iz
        m = self.m
        g = self.g

        # Fetch the states from the newly defined getStates method
        delT, X, Y, xdot, ydot, psi, psidot = self.getStates(timestep, use_slam=True)
        # You must not use true_X, true_Y and true_psi since they are for plotting purpose
        _, true_X, true_Y, _, _, true_psi, _ = self.getStates(timestep, use_slam=False)

        # You are free to reuse or refine your code from P3 in the spaces below.
        forwardindex=80 #"looks ahead" mechanism
        _, closest_index = closestNode(X,Y,trajectory)
        if forwardindex + closest_index >= 8203:
           forwardindex=0

        X_desired = trajectory[closest_index + forwardindex,0]
        Y_desired = trajectory[closest_index + forwardindex,1]
        
        psi_desired = np.arctan2(Y_desired - Y, X_desired - X)
        x_velocity=20 #desired velocity
        
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

        # Return all states and calculated control inputs (F, delta)
        return true_X, true_Y, xdot, ydot, true_psi, psidot, F, delta
