import rclpy
import time
from rclpy.node import Node
import ro45_portalrobot_interfaces.msg as msg


class MoveGripper(Node):
    def __init__(self):
        super().__init__('move_gripper') # type: ignore
        self.target_box0 = [0, 200, 200]
        self.target_box1 = [0, 400, 200]
        self.grippingzone = [600, 600, 0]
        self.state = 0
        self.robPos = [0, 0, 0]
        self.objPos = [0, 0, 0]
        self.id = -1
        self.threshold = 10

        self.posSub = self.create_subscription(
            msg.RobotPos,
            'position_publisher',
            self.callback_robPos,
            10)

        # muss vom Controller als Eingang verarbeitet werden
        self.destPub = self.create_publisher(
            Destination,
            'destination',
            10)

        self.objPosSub = self.create_subscription(
            ObjectPosition,
            'object_position',
            self.callback_objPos,
            10)

        self.ml_out = self.create_subscriber(
            MachineLearning,
            'mlClassification_out',
            self.callback_ML,
            10)

    def callback_robPos(self, msg):
        self.robPos = msg.RobotPos

    def callback_objPos(self, msg):
        self.objPos = msg.position

    def callback_ML(self, msg):
        self.id = msg.id

    def moveGripper(self):
        if self.state == 0:
            self.destPub.publish(0, 0, 0, False)
            self.state = 1
            time.sleep(5)
        if self.state == 1:
            self.destPub.publish((self.grippingzone[0], self.grippingzone[1], self.grippingzone[2], False)
        if self.objPos - self.threshold < self.grippingzone && self.objPos + self.threshold > self.grippingzone:
            self.state = 2
            self.destPub.publish(self.objPos[0], self.objPos[1], -10, True)
            time.sleep(1)
        if self.id == 0 && self.state = 2:
            self.destPub.publish(self.target_box0[0], self.target_box0[1], self.target_box0[2], True)
            if self.robPos == self.target_box0:
                self.destPub.publish(self.target_box0[0], self.target_box0[1], self.target_box0[2], False)
                self.state = 1
        if self.id == 1 && self.state = 2:
            self.destPub.publish(self.target_box1[0], self.target_box1[1], self.target_box1[2], True)
            if self.robPos == self.target_box0:
                self.destPub.publish(self.target_box1[0], self.target_box1[1], self.target_box1[2], False)
                self.state = 1



def main(args=None):
    rclpy.init(args=args)
    move_gripper = MoveGripper()
    rclpy.spin(move_gripper)
    move_gripper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
