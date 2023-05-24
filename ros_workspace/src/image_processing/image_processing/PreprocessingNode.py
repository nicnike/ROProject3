import rclpy 
from std_msgs.msg import Int64
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from custom_interfaces.msg import ImageProcessing, ImageProcessingShape
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from . import VideoProcessing
import cv2

 
class PreprocessingNode(Node):
  """An image subscriber which periodically gets new frames."""
  def __init__(self):
    super().__init__('Preprocessing_Node') # type: ignore
    self.get_logger().info('Initializing started')
    self.first_callBack = True
    self.classification_received = False
    self.classification = 0

    self.subscription = self.create_subscription(
      Image, 
      'image_capture', 
      self.listener_callback, 
      10)
    
    self.publisher = self.create_publisher(
      ImageProcessing,
      'object_information',
      10)

    self.bridge = CvBridge()

    # Create publisher and subscriber for ML node
    self.ml_publisher_ = self.create_publisher(
      ImageProcessingShape,
      'mlClassification_in',
      10)
    self.ml_subscriber_ = self.create_subscription(
      Int64,
      'mlClassification_out',
      self.ml_callback,
      10)

    self.processor = VideoProcessing.VideoProcessing()
    self.get_logger().info('Initializing finished')

  def listener_callback(self, data):
    """This function is called everytime a new message is published on the 'image_capture' topic. """
    #self.get_logger().info('Receiving video frame')
    # Convert ROS Image message to OpenCV image to do other stuff with it afterwards
    current_frame = self.bridge.imgmsg_to_cv2(data)
    if self.first_callBack:
      self.processor.VPInitVideo(current_frame)
      self.first_callBack = False

    self.processor.VPProcessVideo(current_frame)
    shape, _,  self.radius, _ = self.processor.VPCommunicateFeatures()
    self.positionX, self.positionY = self.processor.VPCommunicatePoints()
    # Send preprocessed image to ML node and receive 
    if shape > 4 and self.radius > 100:
      self.send_to_ml_node(self.radius, shape)
    else:
      self.get_logger().info("No object information")

  def send_to_ml_node(self, radius, shape):
    self.get_logger().info('Sending image to ML node' + str(radius) + ' ' + str(shape))
    msg = ImageProcessingShape()
    msg.radius = radius
    msg.shape = shape
    self.ml_publisher_.publish(msg)

  def ml_callback(self, msg):
    # Send object position to Kalman filter node
    object_info = ImageProcessing()
    object_info.header.frame_id = 'map'
    object_info.header.stamp = self.get_clock().now().to_msg()
    object_info.position = Point()
    object_info.position.x = self.positionX
    object_info.position.y = self.positionY
    object_info.radius = self.radius
    object_info.classification = msg.data

    #self.get_logger().info("Publishing object information " + 
    #                       str(object_info.classification) + ' ' 
    #                       + str(object_info.radius) + ' ' 
    #                       + str(object_info.position.x) + ' ' 
    #                       + str(object_info.position.y))
    
    self.publisher.publish(object_info)
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = PreprocessingNode()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  main()

