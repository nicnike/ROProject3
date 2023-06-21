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
        @param video: a normal color video - in this case the prepared homography
        @return videoGray: grayscale video
        """
        videoGray = cv2.cvtColor(video, cv2.COLOR_BGR2GRAY)
        if self.debugMode:
            cv2.imshow("VideoGray", videoGray)
        return videoGray

    def VPConvertToBinary(self, video):
        """!
        Converts given grayscale video to binary using Otsu's Method
        @param video: grayscale video
        @return videoBinary: binary video
        """
        _, videoBinary = cv2.threshold(video, 100, 255, cv2.THRESH_BINARY)

        if self.debugMode:
            cv2.imshow("VideoBinary", videoBinary)
        return videoBinary

    def VPApplyOpening(self, video):
        """!
        Applies morphological operation "Opening" on given binary video
        @param video: binary video
        @return videoOpening: binary video but with the opening operation applied
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
        @param video: normal color video
        @return videoTransformed: video with homography applied still in color
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
        @param video: the binary video with opening operation
        @return shape: length of found contours
        @return objectArea: area of found object - currently not used
        @return radius: min circle radius around found object
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
        @param video: the binary video with opening operation
        @return cornerCount: amount of found corners in given image - currently not used
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
        Coordinate System:
        (0,340)(0,0)
        (320,340)(320,0)
        @param video: the binary video with opening operation
        @return objectX: X coordinate of the highest value after distance transformation
        @return objectY: Y coordinate of the highest value after distance transformation
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
        @param video: the binary video with opening operation
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
        Checks if an object is moving out of the frame and resets the current object values to 0
        @param video: the binary video with opening operation
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

    def VPDetectArucoMarkers(self, video):
        """!
        Detects Aruco markers in given video.
        Uses found corners and ids to display it with own function: VPArucoDisplay
        @param video: normal unprocessed color image
        @return videoDetectedMarkers: color image with drawn aruco frames if enabled otherwise unprocessed
        """
        corners, ids, rejected = self.arucoDetector.detectMarkers(video)
        videoDetectedMarkers = self.VPArucoDisplay(corners, ids, rejected, video)
        return videoDetectedMarkers

    def VPArucoDisplay(self, corners, ids, rejected, video):
        """!
        Uses corners and ids of found Aruco Markers in a video frame to calculate the corner positions.
        Saves the top left corners of 4 markers in the homography variable used as the fromPoints.
        Aruco Markers are rotated so all top left corners point toward the center of a square.
        @param corners: all corners of found aruco markers
        @param ids: ids of found aruco markers starting with 0
        @param rejected: found and considered shapes but not valid markers - no purpose here
        @param video: normal unprocessed color image
        @return video: color image with drawn aruco frames if enabled otherwise unprocessed
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

                if 0 <= markerID <= 3:
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

            return videoDebug

    def VPCompensateHomography(self):
        """!
        Compensates the values in the homography variables for the 1cm white border around the Aruco markers
        as the Contrast is needed for detection.
        The compensation value in pixels is calculated by the distance between 2 aruco markers.
        The distances between the marker top left corners are 8 and 7.5 cm.
        @param
        @return
        """
        self.compensationY = int((self.homographyY[0] - self.homographyY[3]) / 8)
        self.compensationX = int((self.homographyX[1] - self.homographyX[0]) / 7.5)
        self.homographyX[0] = self.homographyX[0] + self.compensationX
        self.homographyY[0] = self.homographyY[0] - self.compensationY
        self.homographyY[1] = self.homographyY[1] - self.compensationY
        self.homographyY[2] = self.homographyY[2] + self.compensationY
        self.homographyX[3] = self.homographyX[3] + self.compensationX
        self.homographyY[3] = self.homographyY[3] + self.compensationY

    def VPGenerateCSVData(self):
        """!
        Generates CSV Data used for machine learning if enabled.
        Output value (cat/unicorn) has to be changed manually in this function.
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
        @return objectX: the object X coordinate
        @return objectY: the object Y coordinate
        """
        return float(self.objectX), float(self.objectY)

    def VPCommunicateFeatures(self):
        """!
        Used for ROS Communication - able to send shape and radius
        @return shape: object shape count
        @return radius: object minimum enclosing radius
        """
        return self.shape, self.radius

    def VPProcessVideo(self, video):
        """!
        Main function that uses all Video Processing steps in correct order.
        Can Generate CSV Data if recordCSV variable is set to True.
        @param video: normal unprocessed color video
        @return opening: the binary video with opening operation
        """
        homography = self.VPHomography(video)
        gray = self.VPConvertToGray(homography)
        binary = self.VPConvertToBinary(gray)
        opening = self.VPApplyOpening(binary)
        self.VPObjectedDetected(opening)
        self.VPGetCorners(opening)
        self.VPGetDistanceTransformation(opening)
        self.VPGetContoursAndArea(opening)
        self.VPObjectValueReset(opening)
        if self.recordCSV:
            self.VPGenerateCSVData()
        if self.debugMode:
            cv2.imshow("videoOriginal", video)
        return opening

    def VPInitVideo(self, video):
        """!
        Init function that detects the 4 Aruco markers and compensate for the white borders.
        @param video: normal unprocessed color video
        @return
        """
        self.VPDetectArucoMarkers(video)
        self.VPCompensateHomography()


def main(args=None):
    '''
    test loop - for manual testing
    '''
    cap = cv2.VideoCapture(0)

    testVideoProcessing = VideoProcessing()

    state = 1
    while cap.isOpened():
        _, video = cap.read()

        if state == 1:
            time.sleep(1)
            testVideoProcessing.VPInitVideo(video)
            state = 0

        newVideo = testVideoProcessing.VPProcessVideo(video)
        print(testVideoProcessing.VPCommunicateFeatures(), testVideoProcessing.VPCommunicatePoints())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()