import rclpy
import time
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from custom_interfaces.msg import ObjectPosition, RobotPosWithGripper


class MoveGripperManually(Node):
    def __init__(self):
        super().__init__('move_gripper_m') # type: ignore
        self.increment = 10
        self.gripperOff = False

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

        self.robPos = RobotPos()
        self.moveXUp()


    def callback_robPos(self, msg):
        self.robPos = msg


    def moveXUp(self):
        move = RobotPosWithGripper()
        move.pos_x = self.robPos.pos_x + self.increment
        move.pos_y = self.robPos.pos_y
        move.pos_z = self.robPos.pos_z
        move.activate_gripper = False
        self.destPub.publish(move)
        self.get_logger().info("Rob at Pos: ")

    # def moveXDown(self):

    # def moveYUp(self):

    # def moveYDown(self):

    # def moveZUp(self):

    # def moveZDown(self):


def main(args=None):
    rclpy.init(args=args)
    move_gripper_m = MoveGripperManually()
    rclpy.spin(move_gripper_m)
    move_gripper_m.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
