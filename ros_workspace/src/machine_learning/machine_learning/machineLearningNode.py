import joblib
import rclpy
from ..resource import const
import os
from rclpy.node import Node
from custom_interfaces.msg import ImageProcessingShape, MachineLearning

# Your machine learning model
myMachineLearningModelIs = "trainedSVMModel.joblib"

# One return value of the machinelearning model prediction.
myReturnValueIs = "unicorn"



class machineLearning:
    '''!
    Load deposited machine learning model
    '''
    def __init__(self):
      file_path = os.path.abspath(const.FILEPATH+myMachineLearningModelIs)
      self.model = joblib.load(file_path)
      self.animalType = myReturnValueIs

    def prediction(self, radius, shape):
        '''!
        Takes radius and shape and makes a prediction about which object it is.
        Returns 1 for unicorn and 0 for cat
        @param radius: Radius of the circle around the object
        @param shape: Captured shape of the object
        @return 1 for unicorn and 0 for cat
        '''
        returnValue = self.model.predict([[shape, radius]])
        if returnValue[0] == self.animalType:
            return 1
        else:
            return 0

class machineLearningNode(Node):
    '''!
    Node that receives values and publishes the classification.
    '''
    def __init__(self):
        super().__init__('ML_Node')  # type: ignore
        queueSize = 10
        self.publisher_ = self.create_publisher(
            MachineLearning,
            'mlClassification_out',
            queueSize)
        self.subscription = self.create_subscription(
            ImageProcessingShape,
            'mlClassification_in',
            self.callback_classification,
            queueSize)
        self.subscription  # prevent unused variable warning
        self.ml = machineLearning()

    def callback_classification(self, msg):
      '''!
      Extracts radius and shape from the message and publishes the classification
      @param msg: Received message
      '''
      classification = self.ml.prediction(msg.radius, msg.shape)
      classification_msg = MachineLearning()
      classification_msg.classification = classification
      classification_msg.id = msg.id
      self.publisher_.publish(classification_msg)


def main(args=None):
  rclpy.init(args=args)
  mlClassifikation = machineLearningNode()
  rclpy.spin(mlClassifikation)
  mlClassifikation.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
