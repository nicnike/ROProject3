import joblib
import rclpy
import os
from rclpy.node import Node
from custom_interfaces.msg import ImageProcessingShape, MachineLearning


class machineLearning:
    def __init__(self):
      svmfile_path = os.path.abspath("src/machine_learning/resource/trainedSVMModel.joblib")
      self.model = joblib.load(svmfile_path)

    def prediction(self, radius, shape):
        returnValue = self.model.predict([[shape, radius]])
        if returnValue == "unicorn":
            return 1
        else:
            return 0

class machineLearningNode(Node):
  def __init__(self):
    super().__init__('ML_Node') # type: ignore
    self.publisher_ = self.create_publisher(
        MachineLearning,
        'mlClassification_out',
        10)
    self.subscription = self.create_subscription(
        ImageProcessingShape,
        'mlClassification_in',
        self.callback_classification,
        10)
    self.subscription  # prevent unused variable warning
    self.ml = machineLearning()

  def callback_classification(self, msg):
      classification = self.ml.prediction(msg.radius, msg.shape)
      classification_msg = MachineLearning()
      classification_msg.classification = classification
      classification_msg.id = msg.id
      self.get_logger().info('Publishing: "%s"' % classification)
      self.publisher_.publish(classification_msg)


def main(args=None):
  rclpy.init(args=args)
  mlClassifikation = machineLearningNode()
  rclpy.spin(mlClassifikation)
  mlClassifikation.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
