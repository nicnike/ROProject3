import rclpy 
from std_msgs.msg import Int64
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from custom_interfaces.msg import ImageProcessing, ImageProcessingShape, MachineLearning
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from . import VideoProcessing
from . import PreprocessingObject

class PreprocessingNode(Node):
  """
  PreprocessingNode subscribes to the image_capture topic to receive image 
  data, processes the images, and publishes object information to the object_information topic.
  """

  def __init__(self):
    """
    Constructor for PreprocessingNode class.
    """
    super().__init__('Preprocessing_Node') # type: ignore
    self.get_logger().info('Initializing started')
    self.first_callBack = True
    self.classification_received = False
    self.classification = 0

    self.subscription = self.create_subscription(
      Image, 
      'image_capture', 
      self.imageCapture_callback, 
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
      MachineLearning,
      'mlClassification_out',
      self.ml_callback,
      10)
    
    self.preprocessingObjectList = []
    self. id = 0
    self.processor = VideoProcessing.VideoProcessing()
    self.get_logger().info('Initializing finished')

  def imageCapture_callback(self, data):
    """
    This function is called everytime a new message is published on the 'image_capture' topic.

    @param data: The image data.
    """    
    current_frame = self.bridge.imgmsg_to_cv2(data)
    if self.first_callBack:
      self.processor.VPInitVideo(current_frame)
      self.first_callBack = False


    self.processor.VPProcessVideo(current_frame)
    shape, radius = self.processor.VPCommunicateFeatures()
    positionX, positionY = self.processor.VPCommunicatePoints()
    # Send preprocessed image to ML node and receive 
    if shape > 4 and radius > 100:
      preObj = PreprocessingObject.PreprocessingObject(self.id, self.get_clock().now().to_msg())
      preObj.shape = shape
      preObj.radius = radius
      preObj.positionX = positionX
      preObj.positionY = positionY
      self.id += 1
      self.send_to_ml_node(preObj.radius, preObj.shape, preObj.id)
      self.preprocessingObjectList.append(preObj)
    


  def send_to_ml_node(self, radius, shape, id):
    """
    Sends the object information to the ML node.

    @param radius: The radius of the object.
    @param shape: The shape of the object.
    @param id: The ID of the object.
    """
    msg = ImageProcessingShape()
    msg.radius = radius
    msg.shape = shape
    msg.id = id
    self.ml_publisher_.publish(msg)

  def ml_callback(self, msg):
    """
    Receives the classification from the ML node and sends the object information to the Kalman filter node.

    @param msg: The classification message.
    """
    for preObj in self.preprocessingObjectList:
      if preObj.id == msg.id:
        object_info = ImageProcessing()
        object_info.position = Point()
        object_info.header.frame_id = 'map'
        object_info.header.stamp = preObj.timestamp
        object_info.position.x = preObj.positionX
        object_info.position.y = preObj.positionY
        object_info.radius = preObj.radius
        object_info.classification = msg.classification    
        self.publisher.publish(object_info)
        self.preprocessingObjectList.remove(preObj)        
        self.get_logger().info('Position x: ' + str(object_info.position.x) + ' Positon y: ' + str(object_info.position.y))
  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = PreprocessingNode()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

