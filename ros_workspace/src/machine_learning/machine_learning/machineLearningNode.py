import joblib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from custom_interfaces.msg import ImageProcessingShape

svmPathFile = "trainedSVMModel.joblib"


class machineLearning:
    def __init__(self, svmPathFile):
       self.model = joblib.load(
            svmPathFile
        )
    def prediction(self,surface_area, radius, shape):
        return self.model.predict([surface_area, radius, shape])

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
    self.ml = machineLearning("placeHolder")

  def callback_classification(self, msg):
      classification = self.ml.prediction(msg.surface_area, msg.radius, msg.shape)
      self.get_logger().info('Publishing: "%s"' % classification)
      self.publisher_.publish(classification)


def main(args=None):
  rclpy.init(args=args)
  mlClassifikation = machineLearningNode()
  rclpy.spin(mlClassifikation)
  mlClassifikation.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
