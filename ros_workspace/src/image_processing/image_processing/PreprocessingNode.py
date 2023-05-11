import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from interfaces.msg import imageProcessing
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
    
    self.publisher = self.create_publisher(
      imageProcessing,
      'object_information',
      10)

    self.bridge = CvBridge()

    # Create publisher and subscriber for ML node
    # self.ml_publisher_ = self.create_publisher(Float64, 'ml_input', 10)
    # self.ml_subscriber_ = self.create_subscription(
    #  Int64,
    #  'ml_output',
    #  self.ml_callback,
    #  10)

  def listener_callback(self, data):
    """This function is called everytime a new message is published on the 'image_capture' topic. """
    self.get_logger().info('Receiving video frame')
    # Convert ROS Image message to OpenCV image to do other stuff with it afterwards
    current_frame = self.bridge.imgmsg_to_cv2(data)

    # preprocess Img here
    # do image stuff

    # Send preprocessed image to ML node and receive classification
    classification = self.send_to_ml_node(current_frame)

    # Send object position to Kalman filter node
    object_info = imageProcessing()
    object_info.header.frame_id = 'map'
    object_info.header.stamp = self.get_clock().now().to_msg()
    object_info.point = Point()
    object_info.point.x = 0.0
    object_info.point.y = 0.0
    object_info.radius = 0.0
    object_info.classification = classification

    self.get_logger().info('Publishing object information' + 
      object_info.classification + ' ' + 
      object_info.radius + ' ' +
      object_info.point.x + ' ' +
      object_info.point.y)

    self.publisher.publish(object_info)

    def send_to_ml_node(self, surface_area, radius, shape):
      #todo: send data to ML node
      return 0

    def ml_callback(self, msg):
       #todo: get classification from ML node
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = PreprocessingNode()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()