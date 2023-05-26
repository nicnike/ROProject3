import cv2
import numpy as np
from skimage.feature import peak_local_max
import time

from . import CSVGeneratorML


class VideoProcessing:
    def __init__(self):
        self.homographyX = [0, 0, 0, 0]
        self.homographyY = [0, 0, 0, 0]
        self.originX = 0
        self.originY = 0
        self.targetHeight = 340
        self.targetWidth = 320
        self.objectX = 0
        self.objectY = 0
        self.shape = 0
        self.objectArea = 0.0
        self.radius = 0.0
        self.cornerCount = 0
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
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        self.compensationX = 0
        self.compensationY = 0
        self.recordCSV = False
        self.debugMode = False
        self.objectDetected = False

    def VPConvertToGray(self, video):
        """!
        Converts given video to grayscale
        @param video
        @return videoGray
        """
        videoGray = cv2.cvtColor(video, cv2.COLOR_BGR2GRAY)
        if self.debugMode:
            cv2.imshow("VideoGray", videoGray)
        return videoGray

    def VPConvertToBinary(self, video):
        """!
        Converts given grayscale video to binary using Otsu's Method
        @param video
        @return videoBinary
        """
        _, videoBinary = cv2.threshold(video, 100, 255, cv2.THRESH_BINARY)

        if self.debugMode:
            cv2.imshow("VideoBinary", videoBinary)
        return videoBinary

    def VPApplyOpening(self, video):
        """!
        Applies morphological operation "Opening" on given binary video
        @param video:
        @return videoOpening
        """
        kernel = np.ones((3, 3), np.uint8)
        videoOpening = cv2.morphologyEx(video, cv2.MORPH_CLOSE, kernel, iterations=10)

        if self.debugMode:
            cv2.imshow("VideoBinary", videoOpening)
        return videoOpening

    def VPHomography(self, video):
        """!
        Applies homography transformation on a given video.
        FromPoints are determined by the 4 top left corners of the Aruco markers.
        ToPoints are determined by the targetHeight and targetWidth
        @param video
        @return videoTransformed
        """
        fromPoints = np.array(list(zip(self.homographyX, self.homographyY)), dtype='float32')
        toPoints = np.array([(self.originX + self.targetWidth, self.originY + self.targetHeight),
                             (self.originX + self.targetWidth, self.originY),
                             (self.originX, self.originY),
                             (self.originX, self.originY + self.targetHeight)], dtype='float32')
        H = cv2.getPerspectiveTransform(fromPoints, toPoints)
        videoTransformed = cv2.warpPerspective(video, H, (self.targetWidth, self.targetHeight))
        return videoTransformed

    def VPGetContoursAndArea(self, video):
        """!
        Takes a binary Video.
        Sets Contours and Area using findContours and approximation of polygonal curves.
        Sets Radius with minEnclosingCircle
        @param video
        @return shape, objectArea, radius
        """
        videoBlurred = cv2.GaussianBlur(video, (5, 5), 0)
        videoEdged = cv2.Canny(videoBlurred, 50, 150)
        contours, _ = cv2.findContours(videoEdged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)

            if area > 1000 and self.objectDetected:
                self.shape = len(approx)
                self.objectArea = float(area)

                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                self.radius = radius

                # cv2.circle(video, center, self.radius, (0, 255, 0), 2)

                # cv2.drawContours(video, [cnt], -1, (0, 255, 0), 3)
        return self.shape, self.objectArea, self.radius

    def VPGetCorners(self, video):
        """!
        Takes a binary Video.
        Sets Corner count using goodFeaturesToTrack
        @param video
        @return cornerCount
        """
        corners = cv2.goodFeaturesToTrack(video, 1000, 0.05, 10)
        if corners is not None:
            corners = np.int0(corners)
            if len(corners) > 15 and self.objectDetected:
                self.cornerCount = len(corners)

            if self.debugMode:
                videoDebug = video.copy()
                for i in corners:
                    x, y = i.ravel()
                    cv2.circle(videoDebug, (x, y), 3, 255, -1)
                cv2.imshow("videoCorners", videoDebug)
        return self.cornerCount

    def VPGetDistanceTransformation(self, video):
        """!
        Takes a binary Video.
        Applies distance transformation algorithm with euclidean distance on detected object,
        calculates the maximum value and set the x and y coordinates of the object.
        @param video
        @return objectX, objectY
        """
        videoDebug = video.copy()
        distanceTransformation = cv2.distanceTransform(video, cv2.DIST_L2, 3)
        try:
            localMax = peak_local_max(distanceTransformation, min_distance=20)
            if self.objectDetected:
                self.objectX = localMax[0, 1]
                self.objectY = localMax[0, 0]
        except:
            pass

        if self.debugMode:
            cv2.circle(videoDebug, (self.objectX, self.objectY), radius=10, color=(0, 0, 255), thickness=-1)
            cv2.imshow("videoDistanceTransformation", videoDebug)

        return self.objectX, self.objectY


    def VPObjectedDetected(self, video):
        """!
        Checks if the first and last row is not in contact with an object while also checking if an object is within
        the frame.
        Checked if the sum of white pixels of 255 is within a certain value range.
        @param video
        @return
        """
        firstRow = video[-1, :].sum()
        lastRow = video[1, :].sum()
        mainFrame = video[1:-2].sum()

        if firstRow < 5000 and lastRow < 5000 and mainFrame > 500000:
            self.objectDetected = True
        else:
            self.objectDetected = False

    def VPObjectValueReset(self, video):
        """!
        Checks if an object is moving out of the frame and resets the current values to 0
        @param video:
        @return
        """
        firstRow = video[-1, :].sum()
        lastRow = video[1, :].sum()

        if firstRow < 5000 and lastRow > 5000:
            self.objectX = 0
            self.objectY = 0
            self.shape = 0
            self.objectArea = 0.0
            self.radius = 0.0
            self.cornerCount = 0

    def VPGetSpeed(self):
        """!
        Returns the speed of an object, measured every 0.5 seconds.
        @param
        @return speed
        """
        if time.time() - self.lastTime > 0.5:
            self.frame2 = self.objectY
            if self.frame2 - self.frame1 < 0:
                self.speed = abs((self.frame2 - self.frame1) / (time.time() - self.lastTime))
            self.frame1 = self.frame2
            self.lastTime = time.time()
        return self.speed

    def VPDetectArucoMarkers(self, video):
        """!
        Detects Aruco markers in given video.
        Uses found corners and ids to display it with own function: VPArucoDisplay
        @param video
        @return videoDetectedMarkers
        """
        corners, ids, rejected = self.arucoDetector.detectMarkers(video)
        videoDetectedMarkers = self.VPArucoDisplay(corners, ids, rejected, video)
        return videoDetectedMarkers

    def VPArucoDisplay(self, corners, ids, rejected, video):
        """!
        Uses corners and ids of found Aruco Markers in a video frame to calculate the corner positions.
        Saves the top left corners of 4 markers in the homography variable used as the fromPoints.
        @param corners, ids, rejected, video
        @return video
        """
        videoDebug = video.copy()
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

                if self.debugMode:
                    cv2.line(videoDebug, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(videoDebug, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(videoDebug, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(videoDebug, bottomLeft, topLeft, (0, 255, 0), 2)
                    cv2.circle(videoDebug, topLeft, 4, (0, 0, 255), -1)
                    cv2.putText(videoDebug, str(markerID), (topRight[0], topRight[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 0, 255), 2)
                    cv2.imshow("videoAruco", videoDebug)

            return video

    def VPCompensateHomography(self):
        """!
        Compensates the values in the homography variables for the 1cm white border around the Aruco markers
        as the Contrast is needed for detection.
        The compensation value in pixels is calculated by the distance between 2 aruco markers.
        @param
        @return
        """
        self.compensationY = int((self.homographyY[0] - self.homographyY[3]) / 8)
        self.compensationX = int((self.homographyX[1] - self.homographyX[0]) / 7.5)
        self.homographyX[0] = self.homographyX[0] + self.compensationX
        self.homographyY[0] = self.homographyY[0] - self.compensationY
        # self.homographyX[1] = self.homographyX[1] - self.compensationX
        self.homographyY[1] = self.homographyY[1] - self.compensationY
        # self.homographyX[2] = self.homographyX[2] - self.compensationX
        self.homographyY[2] = self.homographyY[2] + self.compensationY
        self.homographyX[3] = self.homographyX[3] + self.compensationX
        self.homographyY[3] = self.homographyY[3] + self.compensationY

    def VPGenerateCSVData(self):
        """!
        Generates CSV Data used for machine learning.
        @param
        @return
        """
        if 4 <= self.shape <= 8 and 20000 < self.objectArea < 40000:
            CSVData = [self.shape,
                       self.objectArea,
                       self.radius,
                       self.cornerCount,
                       "cat"]
            CSVGeneratorML.appendToCSV(CSVData)

    def VPCommunicatePoints(self):
        """!
        Used for ROS Communication - able to send object coordinates
        @return objectX, objectY
        """
        return float(self.objectX), float(self.objectY)

    def VPCommunicateFeatures(self):
        """!
        Used for ROS Communication - able to send shape, objectArea, radius, cornerCount
        @return shape, objectArea, radius, cornerCount
        """
        return self.shape, self.objectArea, self.radius, self.cornerCount

    def VPCommunicateSpeed(self):
        """!
        Used for ROS Communication - able to send Speed
        @param
        @return speed
        """
        return self.speed

    def VPProcessVideo(self, video):
        """!
        Main function that uses all Video Processing steps in correct order.
        Can Generate CSV Data if recordCSV variable is set to True.
        @param video
        @return binary
        """
        homography = self.VPHomography(video)
        gray = self.VPConvertToGray(homography)
        binary = self.VPConvertToBinary(gray)
        opening = self.VPApplyOpening(binary)
        self.VPObjectedDetected(opening)
        self.VPGetCorners(opening)
        self.VPGetDistanceTransformation(opening)
        self.VPGetContoursAndArea(opening)
        self.VPGetSpeed()
        self.VPObjectValueReset(opening)
        if self.recordCSV:
            self.VPGenerateCSVData()
        if self.debugMode:
            cv2.imshow("videoOriginal", video)
        return opening

    def VPInitVideo(self, video):
        """!
        Init function that detects the 4 Aruco markers and compensate for the white borders.
        @param video
        @return
        """
        self.VPDetectArucoMarkers(video)
        self.VPCompensateHomography()


def main(args=None):
    '''
    test loop - removed soon
    '''
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("sampleVideo.mp4")

    testVideoProcessing = VideoProcessing()

    state = 1
    while cap.isOpened():
        _, video = cap.read()

        if state == 1:
            time.sleep(1)
            testVideoProcessing.VPInitVideo(video)
            state = 0

        newVideo = testVideoProcessing.VPProcessVideo(video)
        print(testVideoProcessing.VPCommunicateFeatures(), testVideoProcessing.VPCommunicatePoints(),
              testVideoProcessing.VPCommunicateSpeed())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()