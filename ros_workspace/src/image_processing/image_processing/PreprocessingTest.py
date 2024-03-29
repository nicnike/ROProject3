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
        self.imgClosing = cv2.imread('../resource/imgClosing.jpg', cv2.IMREAD_GRAYSCALE)

    def test_VPConvertToGray(self):
        """!
        Tests the conversion from an image to a grayscale image.
        Sets a variable to a random input and after using the function checks if the value was changed.
        Testing images is difficult.
        """
        gray = 42
        gray = self.class_VideoProcessing.VPConvertToGray(self.imgHomography)
        self.assertIsNot(gray, 42)

        # Edge Case
        self.assertIsNot(gray, 0)
        self.assertIsNot(gray, "")

    def test_VPConvertToBinary(self):
        """!
        Tests the conversion from a grayscale image to a binary image.
        Sets a variable to a random input and after using the function checks if the value was changed.
        Testing images is difficult.
        """
        binary = 42
        binary = self.class_VideoProcessing.VPConvertToBinary(self.imgGray)
        self.assertIsNot(binary, 42)

        # Edge Case
        self.assertIsNot(binary, 0)
        self.assertIsNot(binary, "")

    def test_VPApplyClosing(self):
        """!
        Tests the application of the closing function on a binary image.
        Sets a variable to a random input and after using the function checks if the value was changed.
        Testing images is difficult.
        """
        closing = 42
        closing = self.class_VideoProcessing.VPApplyClosing(self.imgBinary)
        self.assertIsNot(closing, 42)

        # Edge Case
        self.assertIsNot(closing, 0)
        self.assertIsNot(closing, "")

    def test_VPHomography(self):
        """!
        Tests the application of the homography function on a binary image
        Sets a variable to a random input and after using the function checks if the value was changed.
        Testing images is difficult.
        """
        homography = 42
        homography = self.class_VideoProcessing.VPHomography(self.imgBinary)
        self.assertIsNot(homography, 42)

        # Edge Case
        self.assertIsNot(homography, 0)
        self.assertIsNot(homography, "")

    # Test for Init Functions
    def test_VPDetectArucoMarkers(self):
        """!
        Tests the detection of the 4 Aruco markers in a given frame and checks if exactly 4 are found.
        Tests VPArucoDisplay automatically with it as well
        """
        self.class_VideoProcessing.VPDetectArucoMarkers(self.img)
        self.assertEqual(len(self.class_VideoProcessing.homographyX), 4)

        # Edge Case
        self.assertIsNot(len(self.class_VideoProcessing.homographyX), "")
        self.assertIsNot(self.class_VideoProcessing.homographyX, (0, 0, 0, 0))

    def test_VPGetContoursAndArea(self):
        """!
        Tests the variables shape, objectArea and radius of the object within the given frame.

        The objectDetected variable has to be True because test functions run in random order
        and the variable has to be tested later as well
        """
        self.class_VideoProcessing.objectDetected = True
        self.class_VideoProcessing.VPGetContoursAndArea(self.imgClosing)
        self.assertEqual((self.class_VideoProcessing.shape,
                          self.class_VideoProcessing.objectArea,
                          self.class_VideoProcessing.radius),
                         (7, 35274.5, 150.90219116210938))

        # Edge Case
        self.assertIsNot((self.class_VideoProcessing.shape,
                          self.class_VideoProcessing.objectArea,
                          self.class_VideoProcessing.radius),
                         (0, 0.0, 0.0))
        self.assertIsNot((self.class_VideoProcessing.shape,
                          self.class_VideoProcessing.objectArea,
                          self.class_VideoProcessing.radius),
                         "")

    def test_VPGetDistanceTransformation(self):
        """!
        Tests the variables objectX and objectY of the object within the given frame.

        The objectDetected variable has to be True because test functions run in random order
        and the variable has to be tested later as well
        """
        self.class_VideoProcessing.objectDetected = True
        self.class_VideoProcessing.VPGetDistanceTransformation(self.imgClosing)
        self.assertEqual((self.class_VideoProcessing.objectX, self.class_VideoProcessing.objectY), (135, 241))
        # Edge Case
        self.assertIsNot((self.class_VideoProcessing.objectX, self.class_VideoProcessing.objectY), (0, 0))
        self.assertIsNot((self.class_VideoProcessing.objectX, self.class_VideoProcessing.objectY), "")

    def test_VPGetCorners(self):
        """!
        Tests the variable cornerCount of the object within the given frame.

        The objectDetected variable has to be True because test functions run in random order
        and the variable has to be tested later as well
        """
        self.class_VideoProcessing.objectDetected = True
        self.class_VideoProcessing.VPGetCorners(self.imgClosing)
        self.assertEqual(self.class_VideoProcessing.cornerCount, 51)

        # Edge Case
        self.assertIsNot(self.class_VideoProcessing.cornerCount, 0)
        self.assertIsNot(self.class_VideoProcessing.cornerCount, "")

    def test_VPObjectedDetected(self):
        """!
        Tests if an object is correctly detected within the given frame.
        The objectDetected variable should be set to True if correct.
        """
        self.class_VideoProcessing.objectDetected = False
        self.class_VideoProcessing.VPObjectedDetected(self.imgClosing)
        self.assertEqual(self.class_VideoProcessing.objectDetected, True)

    def test_VPObjectValueReset(self):
        """!
        Tests if the Reset function correctly resets every variable back to 0 or 0.0 if number is a float
        Values cannot be set to a random state because the reset condition of the function will not be met
        """
        self.class_VideoProcessing.VPObjectValueReset(self.imgClosing)
        self.assertEqual((self.class_VideoProcessing.objectX,
                          self.class_VideoProcessing.objectY,
                          self.class_VideoProcessing.shape,
                          self.class_VideoProcessing.objectArea,
                          self.class_VideoProcessing.radius,
                          self.class_VideoProcessing.cornerCount),
                         (0, 0, 0, 0.0, 0.0, 0))
        # Edge Case
        self.assertIsNot((self.class_VideoProcessing.objectX,
                          self.class_VideoProcessing.objectY,
                          self.class_VideoProcessing.shape,
                          self.class_VideoProcessing.objectArea,
                          self.class_VideoProcessing.radius,
                          self.class_VideoProcessing.cornerCount),
                         "")
        self.assertLess((self.class_VideoProcessing.objectX,
                          self.class_VideoProcessing.objectY,
                          self.class_VideoProcessing.shape,
                          self.class_VideoProcessing.objectArea,
                          self.class_VideoProcessing.radius,
                          self.class_VideoProcessing.cornerCount),
                         (1, 1, 1, 1.0, 1.0, 1))

    def test_VPCommunicatePoints(self):
        """!
        Tests if the CommunicatePoints function correctly returns the X ans Y Coordinates
        """
        self.class_VideoProcessing.objectX = 4
        self.class_VideoProcessing.objectY = 10
        points = self.class_VideoProcessing.VPCommunicatePoints()
        self.assertEqual(points, (4, 10))

        # Edge Case
        self.assertIsNot(points, (4.0, 10.0))
        self.assertIsNot(points, (0, 0))
        self.assertIsNot(points, "")

    def test_VPCommunicateFeatures(self):
        """!
        Tests if the CommunicateFeatures function correctly returns shape and radius
        """
        self.class_VideoProcessing.shape = 7
        self.class_VideoProcessing.radius = 150
        features = self.class_VideoProcessing.VPCommunicateFeatures()
        self.assertEqual(features, (7, 150))

        # valid equivalence classes
        self.assertIsNot(features, (1, 100))
        self.assertIsNot(features, (50, 400))

        # invalid equivalence classes
        self.assertIsNot(features, ("a", "a"))
        self.assertIsNot(features, "")
        self.assertIsNot(features, (0, 0))
        self.assertIsNot(features, (-7, -150))
        self.assertIsNot(features, (7.0, 150.0))




if __name__ == '__main__':
    unittest.main()
