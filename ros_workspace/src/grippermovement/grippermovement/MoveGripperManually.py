import rclpy
import time
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RotPos, RobotCmd
from custom_interfaces.msg import ObjectPosition, RobotPosWithGripper


class MoveGripperManually(Node):
    def __init__(self):
        super().__init__('move_gripper_m') # type: ignore

        self.increment = 10

        self.posSub = self.create_subscription(
            RobotPos,
            'robot_position',
            self.callback_robPos,
            10)

        # muss vom Controller als Eingang verarbeitet werden
        self.destPub = self.create_publisher(
            RobotPosWithGripper,
            'robotposwithgripper',
            10)

        self.moveXUp()


    def callback_robPos(self, msg):
        self.robPos = msg


    def moveXUp(self):
        self.destPub.publish(self.robPos.pos_x + self.increment, self.robPos.pos_y, self.robPos.pos_z, False)
        self.get_logger().info("Rob at Pos: " & self.robPos)
    def moveXDown(self):
        self.destPub.publish(self.robPos.pos_x - self.increment, self.robPos[1], self.robPos[2], False)
        self.get_logger().info("Rob at Pos: " & self.robPos)


    def moveYUp(self):
        self.destPub.publish(self.robPos[0], self.robPos[1] + self.increment, self.robPos[2], False)
        self.get_logger().info("Rob at Pos: " & self.robPos)
    def moveYDown(self):
        self.destPub.publish(self.robPos[0], self.robPos[1] - self.increment, self.robPos[2], False)
        self.get_logger().info("Rob at Pos: " & self.robPos)


    def moveZUp(self):
        self.destPub.publish(self.robPos[0], self.robPos[1], self.robPos[2] + self.increment, False)
        self.get_logger().info("Rob at Pos: " & self.robPos)
    def moveZDown(self):
        self.destPub.publish(self.robPos[0], self.robPos[1], self.robPos[2] - self.increment, False)
        self.get_logger().info("Rob at Pos: " & self.robPos)


def main(args=None):
    rclpy.init(args=args)
    move_gripper_m = MoveGripperManually()
    rclpy.spin(move_gripper_m)
    move_gripper_m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
