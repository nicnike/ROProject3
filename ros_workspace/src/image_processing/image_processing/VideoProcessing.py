import cv2
from cv2 import aruco
import cv2.aruco
import numpy as np
from skimage.feature import peak_local_max
import time

import Animal
import CSVGeneratorML


class VideoProcessing:
    def __init__(self, inputVideo):
        self.video = inputVideo
        self.homographyX = [0, 0, 0, 0]
        self.homographyY = [0, 0, 0, 0]
        self.originX = 0
        self.originY = 0
        self.targetHeight = 240
        self.targetWidth = 300
        self.objectX = 0
        self.speed = 0
        self.lastTime = time.time()
        self.frame1 = 0
        self.frame2 = 0
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }
        self.arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.arucoParams = aruco.DetectorParameters()
        self.arucoDetector = aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        self.compensationX = 0
        self.compensationY = 0

    def VPConvertToGray(self, video):
        videoGray = cv2.cvtColor(video, cv2.COLOR_BGR2GRAY)
        return videoGray

    def VPConvertToBinary(self, video):
        _, videoBinary = cv2.threshold(video, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return videoBinary

    def VPHomography(self, video):
        fromPoints = np.array(list(zip(self.homographyX, self.homographyY)), dtype='float32')
        toPoints = np.array([(self.originX + self.targetWidth, self.originY + self.targetHeight),
                             (self.originX + self.targetWidth, self.originY),
                             (self.originX, self.originY),
                             (self.originX, self.originY + self.targetHeight)], dtype='float32')
        H = cv2.getPerspectiveTransform(fromPoints, toPoints)
        videoTransformed = cv2.warpPerspective(video, H, (self.targetWidth, self.targetHeight))
        return videoTransformed

    def VPGetContoursAndArea(self, video, Animal):
        videoBlurred = cv2.GaussianBlur(video, (5, 5), 0)
        videoEdged = cv2.Canny(videoBlurred, 50, 150)
        contours, _ = cv2.findContours(videoEdged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)

            if area > 1000:
                Animal.shape = len(approx)
                Animal.objectArea = area

                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                Animal.radius = radius
                print(radius)
                cv2.circle(video, center, radius, (0, 255, 0), 2)

                # cv2.drawContours(video, [cnt], -1, (0, 255, 0), 3)


    def VPGetCorners(self, video, Animal):
        corners = cv2.goodFeaturesToTrack(video, 1000, 0.05, 10)
        if corners.any() is not None:
            corners = np.int0(corners)
            Animal.cornerCount = len(corners)

        # for i in corners:
        #     x, y = i.ravel()
        #     cv2. circle(video, (x, y), 3, 255, -1)

    def VPGetDistanceTransformation(self, video, Animal):
        distanceTransformation = cv2.distanceTransform(video, cv2.DIST_L2, 3)
        try:
            localMax = peak_local_max(distanceTransformation, min_distance=20)
            self.objectX = localMax[0, 1]
            Animal.objectX = localMax[0, 1]
            Animal.objectY = localMax[0, 0]
        except:
            pass

        # cv2.circle(video, (x, y), radius=10, color=(0, 0, 255), thickness=-1)

        return Animal.objectX, Animal.objectY

    def VPGetSpeed(self):
        '''
        Returns the speed of an object.
        :parameter: self
        :return: self.speed
        '''
        if time.time() - self.lastTime > 0.5:
            self.frame2 = self.objectX
            if self.frame2 - self.frame1 > 0:
                self.speed = (self.frame2 - self.frame1) / (time.time() - self.lastTime)
            self.frame1 = self.frame2
            self.lastTime = time.time()
        return self.speed

    def VPArucoDisplay(self, corners, ids, rejected, video):
        if len(corners) > 0:
            ids = ids.flatten()

            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                if 0 <= markerID <= 4:
                    self.homographyX[markerID] = int(topLeft[0])
                    self.homographyY[markerID] = int(topLeft[1])

                # cv2.line(video, topLeft, topRight, (0, 255, 0), 2)
                # cv2.line(video, topRight, bottomRight, (0, 255, 0), 2)
                # cv2.line(video, bottomRight, bottomLeft, (0, 255, 0), 2)
                # cv2.line(video, bottomLeft, topLeft, (0, 255, 0), 2)
                # cv2.circle(video, topLeft, 4, (0, 0, 255), -1)
                # cv2.putText(video, str(markerID), (topRight[0], topRight[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            return video

    def VPDetectArucoMarkers(self, video):
        corners, ids, rejected = self.arucoDetector.detectMarkers(video)
        videoDetectedMarkers = self.VPArucoDisplay(corners, ids, rejected, video)
        return videoDetectedMarkers

    def VPCompensateHomography(self):
        self.compensationY = int((self.homographyY[0] - self.homographyY[1]) / 6)
        self.compensationX = int((self.homographyX[0] - self.homographyX[3]) / 8)
        self.homographyX[0] = self.homographyX[0] - self.compensationX
        self.homographyY[0] = self.homographyY[0] - self.compensationY
        self.homographyX[1] = self.homographyX[1] - self.compensationX
        self.homographyY[1] = self.homographyY[1] + self.compensationY
        self.homographyX[2] = self.homographyX[2] + self.compensationX
        self.homographyY[2] = self.homographyY[2] + self.compensationY
        self.homographyX[3] = self.homographyX[3] + self.compensationX
        self.homographyY[3] = self.homographyY[3] - self.compensationY

    def VPGenerateCSVData(self, Animal):
        if Animal.shape != 0:
            CSVData = [Animal.shape,
                       Animal.objectArea,
                       Animal.radius,
                       Animal.cornerCount,
                       str(Animal)]
            CSVGeneratorML.appendToCSV(CSVData)

    def VPCommunicateSpeed(self):
        return self.speed

    def VPProcessVideo(self, video, Animal):
        homography = self.VPHomography(video)
        gray = self.VPConvertToGray(homography)
        self.VPGetCorners(gray, Animal)
        binary = self.VPConvertToBinary(gray)
        self.VPGetDistanceTransformation(binary, Animal)
        self.VPGetContoursAndArea(binary, Animal)
        self.VPGetSpeed()
        return binary


'''
test loop - removed soon
'''
cap = cv2.VideoCapture(0)

testVideoProcessing = VideoProcessing(cap)
animal1 = Animal.Animal()

state = 1
hstate = 1
while cap.isOpened():
    _, video = cap.read()

    if state == 1:
        detectedMarkers = testVideoProcessing.VPDetectArucoMarkers(video)
        state = 0


    newVideo = testVideoProcessing.VPProcessVideo(video, animal1)
    print(animal1.CommunicateFeatures(), animal1.CommunicatePoints(), testVideoProcessing.VPCommunicateSpeed())

    if hstate == 1:
        testVideoProcessing.VPCompensateHomography()
        hstate = 0


    cv2.imshow("ImageOriginal", video)
    cv2.imshow("Image", newVideo)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()