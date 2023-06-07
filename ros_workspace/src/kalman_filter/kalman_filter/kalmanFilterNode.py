import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ObjectPosition, ImageProcessing
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import Point
import numpy as np
import csv


class Object:
  def __init__(self, id, classification, radius, centroid, timestamp):
    """   
    Initializes an Object instance with the given parameters.

    @param id: The ID of the object.
    @param classification: The classification of the object.
    @param radius: The radius of the object.
    @param centroid: The centroid of the object.
    @param timestamp: The timestamp of the object.
    """
    self.id = id
    self.classification = classification
    self.radius = radius
    self.timestamp = timestamp
    self.kf = KalmanFilter(dim_x=4, dim_z=2)
    self.kf.x = np.array([centroid[0], centroid[1], 0, 0.1])
    '''
    The F matrix is the state transition matrix, which describes how the 
    state of the system evolves over time. In this case, the matrix is set 
    to a 4x4 identity matrix, which means that the position and velocity of
    the object are assumed to be constant over time.
    '''
    self.kf.F = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 1],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
    '''
    The H matrix is the measurement matrix, which maps the state vector to 
    the measurement vector. In this case, the matrix is set to a 2x4 matrix 
    that selects the x and y coordinates of the state vector.
    '''
    self.kf.H = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0]])
    '''
    The P matrix is the covariance matrix, which describes the uncertainty
    '''
    self.kf.P *= 1000
    '''
    The R matrix is the measurement noise covariance matrix, which represents 
    the uncertainty in the measurements. In this case, the matrix is set to a 
    2x2 diagonal matrix with values of 0.1, which means that the measurements 
    are assumed to have a standard deviation of 0.1 in both the x and y directions.
    '''
    self.kf.R = np.array([[0.1, 0],
                          [0, 0.1]])
    '''
    The Q matrix is the process noise covariance matrix, which represents the 
    uncertainty in the process that generates the state transitions. In this case, 
    the matrix is set to a 4x4 diagonal matrix with values of 1 and 0.1, 
    which means that the position and velocity of the object are assumed to have 
    a standard deviation of 1 and 0.1, respectively.
    '''
    self.kf.Q = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 0.1, 0],
                          [0, 0, 0, 0.1]])

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
    with open(self.filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['id', 'timestamp', 'x', 'y'])


  def timer_predict(self):
    """
    Predicts the position of each object using the Kalman filter.
    """
    for obj in self.objects:
      sec, nano = self.get_clock().now().seconds_nanoseconds()
      timestamp = sec + nano * 1e-9
      if timestamp - obj.timestamp > 0.1:
        obj.kf.predict(timestamp - obj.timestamp)
        obj.timestamp = timestamp
        # Write the x and y values to the CSV file
        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([obj.id, obj.timestamp, obj.kf.x[0], obj.kf.x[1]])


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
      matched_obj.kf.predict(timestamp - matched_obj.timestamp)
      matched_obj.kf.update(np.array([x, y]))
      matched_obj.timestamp = timestamp
      # Write the x and y values to the CSV file
      with open(self.filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([matched_obj.id, matched_obj.timestamp, matched_obj.kf.x[0], matched_obj.kf.x[1]])
    else:
      obj = Object(self.next_id, classification, radius, np.array([x, y]), timestamp)
      self.objects.append(obj)
      self.next_id += 1
      self.get_logger().info('Creating: "%s"' % obj.id)
      with open(self.filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([obj.id, obj.timestamp, obj.kf.x[0], obj.kf.x[1]])



  # Publish object positions
  def publishCoordinates_timer(self):
    """
    Publishes the position of each object as a ROS2 message.
    """
    for obj in self.objects:
      if obj.kf.x[1] < -400:
        obj.kf.predict(2.0)
        object_position = ObjectPosition()
        object_position.header.frame_id = 'map'
        object_position.header.stamp = self.get_clock().now().to_msg()
        object_position.classification = obj.classification
        object_position.position = Point()
        object_position.position.x = obj.kf.x[0]
        object_position.position.y = obj.kf.x[1]
        object_position.position.z = 0
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