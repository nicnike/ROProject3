from ...kalman_filter.kalman_filter import KalmanFilter
import numpy as np




class Animal:
    def __init__(self):
        # TODO speed for kalman
        self.speed = 0

        self.objectX = 0
        self.objectY = 0
        self.shape = 0
        self.objectArea = 0
        self.cornerCount = 0
        self.radius = 0
        self.mlPrediction = 0
        self.KF = KalmanFilter(0.1, 1, 1, 1, 0.1, 0.1)
        self.kalmanX = 0
        self.kalmanY = 0

        self.KF.setxStdMeas(self.speed)


    def CommunicatePoints(self):
        return self.objectX, self.objectY

    def CommunicateFeatures(self):
        return self.shape, self.objectArea, self.radius, self.cornerCount



    #TODO Update Values


    def animalPositionPredict(self):
        self.kalmanX, self.kalmanY = self.KF.predict()

    def animalPositionPredictUpdate(self):
        self.kalmanX, self.kalmanY = self.KF.predict()
        self.kalmanX, self.kalmanY = self.KF.update(np.array([self.objectX, self.objectY]))






