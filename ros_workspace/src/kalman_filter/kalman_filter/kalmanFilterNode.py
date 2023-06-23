import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ObjectPosition, ImageProcessing
from . import ObjectTracker, FileWriterCSV
from geometry_msgs.msg import Point
import numpy as np

class ObjectTrackerNode(Node):
  def __init__(self):
    """
    Initializes an ObjectTrackerNode instance.
    """
    super().__init__('object_tracker') # type: ignore
    self.objects = []
    self.next_id = 0
    self.publisher_ = self.create_publisher(
      ObjectPosition,
      'object_position',
      10)
    self.subscription = self.create_subscription(
      ImageProcessing,
      'object_information',
      self.callback_classification,
      10)
    self.subscription  # prevent unused variable warning
    self.timer = self.create_timer(0.1, self.timer_predict)
    self.timer = self.create_timer(0.1, self.publishCoordinates_timer)

    # Create a CSV file for all objects
    #Will be saved in workspace 
    self.filename = 'object_positions.csv'
    self.fwCSV = FileWriterCSV.FileWriterCSV(self.filename)


  def timer_predict(self):
    """
    Predicts the position of each object using the Kalman filter.
    """
    for obj in self.objects:
      sec, nano = self.get_clock().now().seconds_nanoseconds()
      timestamp = sec + nano * 1e-9
      if timestamp - obj.timestamp > 0.1 and not obj.locked: 
        obj.predictNextStep(timestamp)
        self.fwCSV.write_data(obj.id, obj.timestamp, obj.kf.x[0], obj.kf.x[1])


  def callback_classification(self, msg):
    """
    Callback function that is called when a new object is detected. 
    Updates the position of the object using the Kalman filter.

    @param msg: The ImageProcessing message containing information about the detected object.
    """    
    radius = msg.radius
    x, y = msg.position.x, msg.position.y
    classification = msg.classification
    timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # Check if object already exists
    matched_obj = None
    if self.objects:
      obj = self.objects[-1]
      dist = np.linalg.norm(obj.kf.x[0:2] - np.array([x, y]))
      if dist < obj.radius:
          self.get_logger().info('Found match: ' + str(dist) + ' < ' + str(obj.radius ))
          matched_obj = obj

    # Update or create object
    if matched_obj is not None:
      matched_obj.predictUpdateStep(timestamp,x,y)
      self.fwCSV.write_data(matched_obj.id, matched_obj.timestamp, matched_obj.kf.x[0], matched_obj.kf.x[1])
    else:
      obj = ObjectTracker.ObjectTracker(self.next_id, classification, radius, np.array([x, y]), timestamp, 0.1)
      self.objects.append(obj)
      self.next_id += 1
      obj.predictUpdateStep(timestamp,x,y)
      self.fwCSV.write_data(obj.id, obj.timestamp, obj.kf.x[0], obj.kf.x[1])

  # Publish object positions
  def publishCoordinates_timer(self):
    """
    Publishes the position of each object as a ROS2 message.
    """
    for obj in self.objects:
      if obj.kf.x[1] < -300:
        self.publishCoordinates(obj)

  def publishCoordinates(self,obj):
        obj.locked = True
        self.get_logger().info('old y: ' + str(obj.kf.x[1]))
        obj.calculateNewFMatrix(1.0)
        obj.kf.predict()
        self.fwCSV.write_data(obj.id, obj.timestamp, obj.kf.x[0], obj.kf.x[1])
        self.get_logger().info('new y: ' + str(obj.kf.x[1]))
        object_position = ObjectPosition()
        object_position.header.frame_id = 'map'
        object_position.header.stamp = self.get_clock().now().to_msg()
        object_position.classification = obj.classification
        object_position.position = Point()
        object_position.position.x = obj.kf.x[0]
        object_position.position.y = obj.kf.x[1]
        object_position.position.z = 0.0
        self.get_logger().info('Publishing: "%s"' % object_position)
        self.publisher_.publish(object_position)
        obj.sendToGripper = True
        self.objects.remove(obj)


def main(args=None):
  rclpy.init(args=args)
  object_tracker = ObjectTrackerNode()
  rclpy.spin(object_tracker)
  object_tracker.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()