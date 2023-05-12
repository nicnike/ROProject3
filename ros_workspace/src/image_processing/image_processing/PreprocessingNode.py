import rclpy 
from std_msgs.msg import Int64
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from custom_interfaces.msg import ImageProcessing, ImageProcessingShape
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import VideoProcessing
 
class PreprocessingNode(Node):
  """An image subscriber which periodically gets new frames."""
  def __init__(self):
    super().__init__('Preprocessing_Node') # type: ignore
    self.get_logger().info('Initializing')

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
    self.processor.VPInitVideo(Image)

  def listener_callback(self, data):
    """This function is called everytime a new message is published on the 'image_capture' topic. """
    self.get_logger().info('Receiving video frame')
    # Convert ROS Image message to OpenCV image to do other stuff with it afterwards
    current_frame = self.bridge.imgmsg_to_cv2(data)

    # preprocess Img here
    # do image stuff
    surface_area = 0
    radius = 0
    shape = 0
    corners = 0

    self.processor.VPProcessVideo(current_frame)
    shape, surface_area, radius, corners = self.processor.VPCommunicateFeatures()
    positionX, positionY = self.processor.VPCommunicatePoints()



    # Send preprocessed image to ML node and receive classification
    classification = self.send_to_ml_node(surface_area, radius, shape, corners)

    # Send object position to Kalman filter node
    object_info = ImageProcessing()
    object_info.header.frame_id = 'map'
    object_info.header.stamp = self.get_clock().now().to_msg()
    object_info.position = Point()
    object_info.position.x = 0.0
    object_info.position.y = 0.0
    object_info.radius = 0.0
    object_info.corners = 0.0
    object_info.classification = classification

    self.get_logger().info("Publishing object information " + 
                           str(object_info.classification) + ' ' 
                           + str(object_info.radius) + ' ' 
                           + str(object_info.position.x) + ' ' 
                           + str(object_info.position.y))
    
    self.publisher.publish(object_info)

  def send_to_ml_node(self, surface_area, radius, shape):
    msg = ImageProcessingShape()
    msg.surface_area = surface_area
    msg.radius = radius
    msg.shape = shape
    self.publisher.publish(msg)

    # Wait for response from machine learning node
    while not self.classification_received:
        rclpy.spin_once(self)

    # Reset classification_received flag for next call
    self.classification_received = False

    return self.classification

  def ml_callback(self, msg):
    self.classification_received = True
    self.classification = msg.data
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = PreprocessingNode()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()