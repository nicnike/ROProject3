import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class CameraNode(Node):
    """An image publisher which periodically publishes new frames."""
    def __init__(self, test_mode=True):
        super().__init__('CameraNode') # type: ignore
        self.publisher_ = self.create_publisher(
            Image, 
            'image_capture', 
            10)
        
        timer_period = 0.5  # 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

        #initialize video feed
        if not test_mode:
            self.videoFeed = cv2.VideoCapture(0)
        else:
            self.videoFeed = cv2.VideoCapture('/home/joemama/Videos/sample_video.mp4')

        
    def timer_callback(self):
        ret, frame = self.videoFeed.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from the camera.')
            return
        img_msg = self.bridge.cv2_to_imgmsg(frame)
        self.publisher_.publish(img_msg)
        self.get_logger().info('Publishing a video frame')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = CameraNode()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()