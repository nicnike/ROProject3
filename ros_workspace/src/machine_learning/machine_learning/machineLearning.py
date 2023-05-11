import joblib
import rclpy
from rclpy.node import Node
from interfaces.msg import image_processing_shape, imageProcessing

svmPathFile = "trainedSVMModel.joblib"


class machineLearning:
    def __init__(self, svmPathFile):
       joblib.load(
            svmPathFile
        )
    def prediction(self,surface_area, radius, shape):
        prediction = self.prediction(
          surface_area,
          radius,
          shape)
        return prediction

class machineLearningNode(Node):
  def __init__(self):
    super().__init__('ML_Node')
    self.publisher_ = self.create_publisher(
        imageProcessing,
        'machine_learning_classification',
        10)
    self.subscription = self.create_subscription(
        image_processing_shape,
        'mlClassification',
        self.callback_classification,
      10)
    self.subscription  # prevent unused variable warning

  def callback_classification(self, msg):
      surface_area = msg.surface_area
      radius = msg.radius
      shape = msg.shape
      classification = machineLearning.prediction(
          surface_area,
          radius,
          shape)
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
