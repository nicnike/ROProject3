import rclpy
from rclpy.node import Node
from interfaces.msg import objectPosition
from interfaces.msg import imageProcessing
from filterpy.kalman import KalmanFilter
import numpy as np

class Object:
  def __init__(self, id, classification, radius, centroid, timestamp):
    self.id = id
    self.classification = classification
    self.radius = radius
    self.centroid = centroid
    self.timestamp = timestamp
    self.kf = KalmanFilter(dim_x=4, dim_z=2)
    self.kf.x = np.array([centroid[0], centroid[1], 0, 0])
    self.kf.F = np.array([[1, 0, 1, 0],
                          [0, 1, 0, 1],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
    self.kf.H = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0]])
    self.kf.P *= 1000
    self.kf.R = np.array([[0.1, 0],
                          [0, 0.1]])
    self.kf.Q = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 0.1, 0],
                          [0, 0, 0, 0.1]])

class ObjectTrackerNode(Node):
  def __init__(self):
    super().__init__('object_tracker')
    self.objects = []
    self.next_id = 0
    self.publisher_ = self.create_publisher(
      objectPosition,
      'object_position',
      10)
    self.subscription = self.create_subscription(
      imageProcessing,
      'object_information',
      self.callback_classification,
      10)
    self.subscription  # prevent unused variable warning

  def callback_classification(self, msg):
    # Get object classification, radius, and centroid from message
    radius = msg.radius
    x, y = msg.position
    classification = msg.classification
    timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # Check if object already exists
    matched_obj = None
    for obj in self.objects:
      dist = np.linalg.norm(obj.centroid - np.array([x, y]))
      if dist < obj.radius:
        matched_obj = obj
        break

    # Update or create object
    if matched_obj is not None:
      matched_obj.kf.predict(timestamp - matched_obj.timestamp)
      matched_obj.kf.update(np.array([x, y]))
      matched_obj.centroid = np.array([x, y])
      matched_obj.timestamp = timestamp
    else:
      obj = Object(self.next_id, classification, radius, np.array([x, y]), timestamp)
      self.objects.append(obj)
      self.next_id += 1

    # Publish object position as ROS2 message
    for obj in self.objects:
      obj.kf.predict(timestamp - obj.timestamp)
      obj.timestamp = timestamp
      object_position = objectPosition()
      object_position.header.frame_id = 'map'
      object_position.header.stamp = self.get_clock().now().to_msg()
      object_position.classification = obj.classification
      object_position.point = Point()
      object_position.point.x = obj.kf.x[0]
      object_position.point.y = obj.kf.x[1]
      object_position.point.z = 0
      self.get_logger().info('Publishing: "%s"' % object_position)
      self.publisher_.publish(object_position)

def main(args=None):
  rclpy.init(args=args)
  object_tracker = ObjectTrackerNode()
  rclpy.spin(object_tracker)
  object_tracker.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()