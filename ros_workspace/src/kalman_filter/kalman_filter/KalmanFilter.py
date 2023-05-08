import numpy as np
class KalmanFilter(object):
    def __init__(self, dt, uX, uY, stdAcc, xStdMeas, yStdMeas):
        """
               :param dt: sampling time (time for 1 cycle)
               :param uX: acceleration in x-direction
               :param uY: acceleration in y-direction
               :param stdAcc: process noise magnitude
               :param xStdMeas: standard deviation of the measurement in x-direction
               :param yStdMeas: standard deviation of the measurement in y-direction
               """
        #Define sampling time
        self.dt = dt

        #Define x standard deviation
        self.xStdMeas = xStdMeas

        #Define y standard deviation
        self.yStdMeas = yStdMeas

        #Define the control input variables
        self.u = np.matrix([[uX], [uY]])

        #Initial State
        self.x = np.matrix([[0], [0], [0], [0]])

        #Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        #Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt, 0],
                            [0, self.dt]])

        #Define Measurment Mapping Matrix
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        #Initial Process Noise Convariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * stdAcc ** 2

        #Initial Measurment Noise Covariance
        self.R = np.matrix([[self.xStdMeas ** 2, 0],
                            [0, self.yStdMeas ** 2]])

        #Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1]) #eye= unity  Matrix

    def predict (self):
        #update time state
        #x_k = Ax_(k-1) +Bu_(k-1)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        #Calculate error covariance
        # P = A*P*A` + Q  A´= Transponiert
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:2]

    def update(self, z): #z = measurement
        #S = H*P*H´+P
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        #Calculate the Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #

        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))
        I = np.eye(self.H.shape[1])

        #Update error covariance Matrix
        self.P = (I - (K * self.H)) * self.P
        return self.x[0:2]

    def setxStdMeas(self, newXStdMeas):
        self.xStdMeas = newXStdMeas

    def setyStdMeas(self, newYStdMeas):
        self.yStdMeas = newYStdMeas



