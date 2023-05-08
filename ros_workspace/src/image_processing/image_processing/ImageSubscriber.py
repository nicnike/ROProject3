import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
 
class ImageSubscriber(Node):
  """An image subscriber which periodically gets new frames."""
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      'image_capture', 
      self.listener_callback, 
      10)
    self.subscription 
    self.bridge = CvBridge()

  def listener_callback(self, data):
    """This function is called everytime a new message is published on the 'image_capture' topic. """
    self.get_logger().info('Receiving video frame')
    # Convert ROS Image message to OpenCV image to do other stuff with it afterwards
    current_frame = self.bridge.imgmsg_to_cv2(data)

    #Do Image stuff here

    #Print image shape
    self.get_logger().info('Image shape: %s' % str(current_frame.shape))
    # cv2.imshow("camera", current_frame)
    # cv2.waitKey(1)
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()