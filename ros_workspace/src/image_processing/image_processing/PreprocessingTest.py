import unittest
import cv2
import VideoProcessing


class TestVideoProcessing(unittest.TestCase):

    def setUp(self):
        self.class_VideoProcessing = VideoProcessing.VideoProcessing()
        self.img = cv2.imread('../resource/imgOriginal.jpg')
        self.imgHomography = cv2.imread('../resource/imgHomography.jpg')
        self.imgGray = cv2.imread('../resource/imgGray.jpg')
        self.imgBinary = cv2.imread('../resource/imgBinary.jpg')
        self.imgOpening = cv2.imread('../resource/imgOpening.jpg', cv2.IMREAD_GRAYSCALE)

    def test_VPConvertToGray(self):
        gray = 42
        gray = self.class_VideoProcessing.VPConvertToGray(self.imgHomography)
        self.assertIsNot(gray, 42)

    def test_VPConvertToBinary(self):
        binary = 42
        binary = self.class_VideoProcessing.VPConvertToBinary(self.imgGray)
        self.assertIsNot(binary, 42)

    def test_VPApplyOpening(self):
        opening = 42
        opening = self.class_VideoProcessing.VPApplyOpening(self.imgBinary)
        self.assertIsNot(opening, 42)

    def test_VPHomography(self):
        homography = 42
        homography = self.class_VideoProcessing.VPHomography(self.imgBinary)
        self.assertIsNot(homography, 42)

    # Test for Init Functions
    # tests VPArucoDisplay automatically with it as well
    def test_VPDetectArucoMarkers(self):
        self.class_VideoProcessing.VPDetectArucoMarkers(self.img)
        self.assertEqual(len(self.class_VideoProcessing.homographyX), 4)

    def test_VPGetContoursAndArea(self):
        self.class_VideoProcessing.objectDetected = True
        self.class_VideoProcessing.VPGetContoursAndArea(self.imgOpening)
        self.assertEqual((self.class_VideoProcessing.shape, self.class_VideoProcessing.objectArea, self.class_VideoProcessing.radius), (7, 35274.5, 150.90219116210938))

    def test_VPGetDistanceTransformation(self):
        self.class_VideoProcessing.objectDetected = True
        self.class_VideoProcessing.VPGetDistanceTransformation(self.imgOpening)
        self.assertEqual((self.class_VideoProcessing.objectX, self.class_VideoProcessing.objectY), (135, 241))

    def test_VPGetCorners(self):
        self.class_VideoProcessing.objectDetected = True
        self.class_VideoProcessing.VPGetCorners(self.imgOpening)
        self.assertEqual(self.class_VideoProcessing.cornerCount, 51)

    def test_VPObjectedDetected(self):
        self.class_VideoProcessing.VPObjectedDetected(self.imgOpening)
        self.assertEqual(self.class_VideoProcessing.objectDetected, True)

    def test_VPObjectValueReset(self):
        self.class_VideoProcessing.VPObjectValueReset(self.imgOpening)
        self.assertEqual((self.class_VideoProcessing.objectX,
                          self.class_VideoProcessing.objectY,
                          self.class_VideoProcessing.shape,
                          self.class_VideoProcessing.objectArea,
                          self.class_VideoProcessing.radius,
                          self.class_VideoProcessing.cornerCount),
                         (0, 0, 0, 0.0, 0.0, 0))

    def test_VPCommunicatePoints(self):
        self.class_VideoProcessing.objectX = 4
        self.class_VideoProcessing.objectY = 10
        points = self.class_VideoProcessing.VPCommunicatePoints()
        self.assertEqual(points, (4, 10))

    def test_VPCommunicateFeatures(self):
        self.class_VideoProcessing.shape = 7
        self.class_VideoProcessing.objectArea = 35000
        self.class_VideoProcessing.radius = 150
        self.class_VideoProcessing.cornerCount = 50
        features = self.class_VideoProcessing.VPCommunicateFeatures()
        self.assertEqual(features, (7, 35000, 150, 50))

    def test_VPCommunicateSpeed(self):
        self.class_VideoProcessing.speed = 100
        speed = self.class_VideoProcessing.VPCommunicateSpeed()
        self.assertEqual(speed, 100)

    # ToDo: keine Ahnung wie das zu Testen ist
    # def test_VPCompensateHomography(self):
    # def test_VPGetSpeed(self):
    # def test_VPProcessVideo(self):


if __name__ == '__main__':
    unittest.main()
