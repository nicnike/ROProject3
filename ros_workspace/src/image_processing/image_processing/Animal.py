class Animal:
    def __init__(self):
        self.objectX = 0
        self.objectY = 0
        self.shape = 0
        self.objectArea = 0
        self.cornerCount = 0
        self.radius = 0
        self.mlPrediction = 0
        self.kalmanX = 0
        self.kalmanY = 0

    def CommunicatePoints(self):
        return self.objectX, self.objectY

    def CommunicateFeatures(self):
        return self.shape, self.objectArea, self.radius, self.cornerCount
