import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
 
class PreprocessingNode(Node):
  """An image subscriber which periodically gets new frames."""
  def __init__(self):
    super().__init__('Preprocessing_Node')
    self.get_logger().info('Initializing')
    self.subscription = self.create_subscription(
      Image, 
      'image_capture', 
      self.listener_callback, 
      10)
    
    self.publisher_ = self.create_publisher(
      Image,
      'preprocessed_image',
      10)

    self.bridge = CvBridge()

  def listener_callback(self, data):
    """This function is called everytime a new message is published on the 'image_capture' topic. """
    self.get_logger().info('Receiving video frame')
    # Convert ROS Image message to OpenCV image to do other stuff with it afterwards
    current_frame = self.bridge.imgmsg_to_cv2(data)

    # preprocess Img here
    # do image stuff

    preprocessed_msg = self.bridge.cv2_to_imgmsg(current_frame)
    self.publisher_.publish(preprocessed_msg)
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = PreprocessingNode()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()