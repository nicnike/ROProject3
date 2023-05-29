import rclpy
from rclpy.node import Node
import ro45_portalrobot_interfaces.msg as msg


class MoveGripper(Node):
    def __init__(self):
        super().__init__('move_gripper') # type: ignore
        self.target_box0 = [0,200,0]
        self.target_box1 = [0, 400, 0]
        self.grippingzone = [600, 600, 0]
        self.state = 0

        self.posSub = self.create_subscription(
            msg.RobotPos,
            'position_publisher',
            self.move_gripper,
            10)

        self.velPub = self.create_publisher(
            msg.RobotCmd,
            'command_subscriber',
            10)

        self.objPos = self.create_subscription(
            ObjectPosition,
            'object_position',
            10)

        # self.pubToController = self.create_publisher(
        #     Pos,
        #     'move_to_pos',
        #     10)

        self.ml_out = self.create_subscriber(
            MachineLearning,
            'mlClassification_out',
            10)

    def moveGripper(self):
        if self.state == 0:
            self.velPub.publish(
                msg.RobotCmd(0, 0, 0, False))
            self.state = 1
        if self.state == 1:
            self.velPub.publish(
                msg.RobotCmd(self.grippingzone[0], self.grippingzone[1], self.grippingzone[2], False))
        if self.objPos.position == self.grippingzone:
            self.state = 1
            self.velPub.publish(
                msg.RobotCmd(self.grippingzone[0], self.grippingzone[1], self.grippingzone[2], True))
            if self.ml_out.id == 0:
                self.velPub.publish(
                    msg.RobotCmd(self.target_box0[0], self.target_box0[1], self.target_box0[2], True))
                if self.objPos.position == self.target_box0:
                    self.velPub.publish(
                        msg.RobotCmd(self.target_box0[0], self.target_box0[1], self.target_box0[2], False))
                    self.state = 0
            if self.ml_out.id == 1:
                self.velPub.publish(
                    msg.RobotCmd(self.target_box1[0], self.target_box1[1], self.target_box1[2], True))
                if self.objPos.position == self.target_box0:
                    self.velPub.publish(
                        msg.RobotCmd(self.target_box1[0], self.target_box1[1], self.target_box1[2], False))
                    self.state = 0



def main(args=None):
    rclpy.init(args=args)
    move_gripper = MoveGripper()
    rclpy.spin(move_gripper)
    move_gripper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
