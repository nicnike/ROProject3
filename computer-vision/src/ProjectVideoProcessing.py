import cv2
from cv2 import aruco
import numpy as np
from skimage.feature import peak_local_max
import time

import CSVGeneratorML


ARUCO_DICT = {
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

arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
arucoParams = aruco.DetectorParameters()

histPx = [0, 0, 0, 0]
histPy = [0, 0, 0, 0]
height = 120
width = 320
# height = 73
# width = 400
x0, y0 = (0, 0)


def getPreprocessedVideo(video):
    fromPts = np.array(list(zip(histPx, histPy)), dtype='float32')
    toPts = np.array([(x0, y0 + height), (x0 + width, y0 + height), (x0 + width, y0), (x0, y0)], dtype='float32')

    H = cv2.getPerspectiveTransform(fromPts, toPts)

    videoTransformed = cv2.warpPerspective(video, H, (width + 2 * x0, height + 2 * y0))
    # convert to Gray
    videoGray = cv2.cvtColor(videoTransformed, cv2.COLOR_BGR2GRAY)
    # convert to Binary
    _, videoBinary = cv2.threshold(videoGray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    return videoBinary


'''
Function to evaluate an object within a video frame.
It will return the shape and area of a eligible object.
'''


def getContours(video):
    shape = 0
    area = 0
    areaObject = 0

    videoBlurred = cv2.GaussianBlur(video, (5, 5), 0)
    videoEdged = cv2.Canny(videoBlurred, 50, 150)
    contours, _ = cv2.findContours(videoEdged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(cnt)
        if area > 1000:
            shape = len(approx)
            areaObject = area
            #cv2.drawContours(video, [cnt], -1, (255, 255, 255), 3)
            #cv2.putText(video, str(shape), (cnt[0][0][0], cnt[0][0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    return shape, areaObject


'''
Calculates the local Maximum values in a given preprocessed video frame.
Returns the x and y value of these
'''


def getDistanceTransformation(video):
    x, y = 0, 0
    distanceTransformation = cv2.distanceTransform(video, cv2.DIST_L2, 3)
    try:
        localMax = peak_local_max(distanceTransformation, min_distance=20)
        x = localMax[0, 1]
        y = localMax[0, 0]
    except:
        print("no local max")

    #videoDistanceTransformation = cv2.cvtColor(video, cv2.COLOR_GRAY2RGB)
    #cv2.circle(videoDistanceTransformation, (x, y), radius=10, color=(0, 0, 255), thickness=-1)
    #cv2.imwrite("dist_transform.png", distanceTransformation)

    return x, y


'''
Calculates the Speed of the Object in a video frame.
The X Position will be updated every 0.2 seconds and is only valid if the position change is positive
so there will be no speed change if a new object enters from the left side of the frame.

The Function takes the x parameter from the created local maximum of the getDistanceTransformation function.

x1, x2, speed, lastTime has to be initialized outside of the function.
'''


x1, x2, speed = 0, 0, 0
lastTime = time.time()


def getSpeed(x):
    global x1, x2, speed, lastTime

    if time.time() - lastTime > 0.2:
        x2 = x
        if x2 - x1 > 0:
            speed = (x2 - x1) / (time.time() - lastTime)
        x1 = x2
        lastTime = time.time()

    return speed


'''
aruco display and filling in top left corner values
'''


def arucoDisplay(corners, ids, rejected, image):
    if len(corners) > 0:
        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):

            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = ( int( topRight[0] ), int( topRight[1] ) )
            bottomRight = ( int( bottomRight[0] ), int( bottomRight[1] ) )
            bottomLeft = ( int( bottomLeft[0] ), int( bottomLeft[1] ) )
            topLeft = ( int( topLeft[0] ), int( topLeft[1] ) )

            if 0 <= markerID <= 4:
                histPx[markerID] = int( topLeft[0] )
                histPy[markerID] = int( topLeft[1] )

            # cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            # cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            # cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            # cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            #
            # cv2.circle(image, topLeft, 4, (0, 0, 255), -1)
            #
            # cv2.putText(image, str(markerID), (topRight[0], topRight[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return image


'''
main loop
'''
cap = cv2.VideoCapture(0)

while cap.isOpened():
    _, img = cap.read()

    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

    detectedMarkers = arucoDisplay(corners, ids, rejected, img)

    videoPreprocessed = getPreprocessedVideo(img)

    shape, area = getContours(videoPreprocessed)

    x, y = getDistanceTransformation(videoPreprocessed)

    speed = getSpeed(x)

    print(shape, area, x, y, speed)

    if shape is not 0:
        CSVData = [shape, area, "Cat"]
        CSVGeneratorML.appendToCSV(CSVData)

    cv2.imshow("Image", detectedMarkers)
    cv2.imshow("Video Preprocessed", videoPreprocessed)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()