import joblib
import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import Int64
from custom_interfaces.msg import ImageProcessingShape


class machineLearning:
    def __init__(self):
      svmfile_path = os.path.abspath("src/machine_learning/resource/trainedSVMModel.joblib")
      self.model = joblib.load(svmfile_path)

    def prediction(self, radius, shape):
        returnValue = self.model.predict([[radius, shape]])
        if returnValue == "Unicorn":
            return 1
        else:
            return 0

class machineLearningNode(Node):
  def __init__(self):
    super().__init__('ML_Node') # type: ignore
    self.publisher_ = self.create_publisher(
        Int64,
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
      if msg.shape > 4 and msg.radius > 100:
        classification = self.ml.prediction(msg.radius, msg.shape)
        classification_msg = Int64()
        classification_msg.data = classification
        self.get_logger().info('Publishing: "%s"' % classification)
        self.publisher_.publish(classification_msg)
      else:
          self.get_logger().info("No object information")

def main(args=None):
  rclpy.init(args=args)
  mlClassifikation = machineLearningNode()
  rclpy.spin(mlClassifikation)
  mlClassifikation.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
