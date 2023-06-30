from filterpy.kalman import KalmanFilter
import numpy as np
import csv

class ObjectTracker:
  def __init__(self, id, classification, radius, centroid, timestamp, dt):
    """!
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
    self.dt = dt
    self.locked = False
    '''
    The F matrix is the state transition matrix, which describes how the 
    state of the system evolves over time. In this case, the matrix is set 
    to a 4x4 identity matrix, which means that the position and velocity of
    the object are assumed to be constant over time.
    '''
    self.kf.F = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0.1],
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
    
  def predictNextStep(self,timestamp):
    """!
    Predicts the next state of the object.
    
    @param timestamp: The timestamp of the object.
    """
    if timestamp - self.timestamp > 0.1:
      self.calculateNewFMatrix(timestamp - self.timestamp)
      self.kf.predict()
      self.timestamp = timestamp

  def predictUpdateStep(self, timestamp,x,y):
      """!
      Predicts and updates the next state of the object. 

      @param timestamp: The timestamp of the object.
      @param x: The x coordinate of the object.
      @param y: The y coordinate of the object.
      """
      self.calculateNewFMatrix(timestamp - self.timestamp)
      self.kf.predict()
      self.kf.update(np.array([x, y]))
      self.timestamp = timestamp

  def calculateNewFMatrix(self, dt):
    self.kf.F = np.array([[1, 0, 0, 0],
                          [0, 1, 0, dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])