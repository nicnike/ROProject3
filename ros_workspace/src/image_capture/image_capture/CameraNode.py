import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import threading

class CameraNode(Node):
    """An image publisher which periodically publishes new frames."""
    def __init__(self):
        super().__init__('CameraNode') # type: ignore
        self.publisher_ = self.create_publisher(
            Image, 
            'image_capture', 
            10)
        
        self.test_mode = False
        timer_period = 0.1  # 0.5 seconds
        self.current_frame = None
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()


        # Start a separate thread to read frames from the video feed
        self.frame_thread = threading.Thread(target=self.read_frames)
        self.frame_thread.start()

    def read_frames(self):
        #initialize video feed
        if not self.test_mode:
            self.videoFeed = cv2.VideoCapture(0)
        else:
            self.videoFeed = cv2.VideoCapture('/home/johndoe/Videos/sample_video.mp4')

        while True:
            ret, frame = self.videoFeed.read()
            if not ret:
                self.get_logger().warn('Failed to capture frame from the camera.')
                break
            self.current_frame = frame
            cv2.imshow('Received Frame', self.current_frame)
            cv2.waitKey(1)

        
    def timer_callback(self):
        if self.current_frame is not None:
            img_msg = self.bridge.cv2_to_imgmsg(self.current_frame)
            self.publisher_.publish(img_msg)
            #self.get_logger().info('Publishing a video frame')


def main(args=None):
    rclpy.init(args=args)
    image_publisher = CameraNode()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()
    #shut down video feed
    image_publisher.videoFeed.release()

if __name__ == '__main__':
    main()