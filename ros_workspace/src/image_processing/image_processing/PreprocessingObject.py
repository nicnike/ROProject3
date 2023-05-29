class PreprocessingObject():
  """
  PreprocessingObject is a class that represents an object detected in an image 
  by the PreprocessingNode. It stores information about the object's ID, classification, 
  timestamp, shape, radius, and position.
  """
  def __init__(self, id, timestamp):
    """
    Constructor for PreprocessingObject class.

    @param id: The ID of the object.
    @param timestamp: The timestamp of the object.
    """
    self.id = id
    self.classification = -1
    self.timestamp = timestamp
    self.shape = 0
    self.radius = 0.0
    self.positionX = 0.0
    self.positionY = 0.0
 