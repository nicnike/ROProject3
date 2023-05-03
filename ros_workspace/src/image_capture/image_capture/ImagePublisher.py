import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class ImagePublisher(Node):
    """An image publisher which periodically publishes new frames."""
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_capture', 10)
        timer_period = 0.5  # 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.videoFeed = cv2.VideoCapture(0)
        
    def timer_callback(self):
        ret, frame = self.videoFeed.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame)
            self.publisher_.publish(msg)
        self.get_logger().info('Publishing a video frame')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()