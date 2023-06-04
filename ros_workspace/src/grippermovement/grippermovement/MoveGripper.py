import rclpy
import time
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from custom_interfaces.msg import ObjectPosition, RobotPosWithGripper


class MoveGripper(Node):
    def __init__(self):
        super().__init__('move_gripper') # type: ignore
        self.target_box0 = [0, 0.2, 0.2]
        self.target_box1 = [0, 0.4, 0.2]
        self.grippingzone = [0.6, 0.6, 0]
        self.grippingzoneInPx = [-400, 135, 0]
        self.down = -0.1
        self.offsetPos = [0, 0, 0]
        self.initmove = [-1, -1, -1]
        self.robPos = [0, 0, 0]
        self.objPos = [0, 0, 0]
        self.threshold = 10
        self.id = -1
        self.init = False
        self.gripping = False
        self.working = False
        self.gripperOn = True
        self.gripperOff = False

        self.timer = self.create_timer(0.01, self.timer_callback)

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

        self.objPosSub = self.create_subscription(
            ObjectPosition,
            'object_position',
            self.callback_objPos,
            10)

    def timer_callback(self):
        self.moveGripper()

    def callback_robPos(self, msg):
        self.robPos = [msg.pos_x, msg.pos_y, msg.pos_z]


    def callback_objPos(self, msg):
        self.objPos = [msg.position.x, msg.position.y, msg.position.z]
        self.id = msg.classification

    def calcOffsetPos(self):
        self.offsetPos = self.grippingzone + (self.objPos - self.grippingzoneInPx) * 2.5  # /40 * 100 (40px = 1cm zu m)

    def moveGripper(self):
        if not self.init:
            self.get_logger().info("Initialization...")
            self.destPub.publish(-1, -1, -1, False)
            self.init = True
            self.gripping = True
            time.sleep(10)
            self.get_logger().info("Initialization done")

        if self.gripping:
            self.get_logger().info("Moving to GrippingZone")
            self.destPub.publish(self.grippingzone[0], self.grippingzone[1], self.grippingzone[2], self.gripperOff)
        if self.objPos - self.threshold < self.grippingzoneInPx or self.objPos + self.threshold > self.grippingzoneInPx:
            self.get_logger().info("Picking Object...")
            self.calcOffsetPos()
            self.destPub.publish(self.offsetPos[0], self.offsetPos[1], self.offsetPos[2], self.gripperOff)
            time.sleep(2)
            self.destPub.publish(self.offsetPos[0], self.offsetPos[1], self.down, self.gripperOn)
            time.sleep(0.2)
            self.gripping = False
            self.working = True
            self.get_logger().info("Object picked")

        if self.working and self.id == 0:
            self.get_logger().info("Sorting Object...")
            self.destPub.publish(self.target_box0[0], self.target_box0[1], self.target_box0[2], self.gripperOn)
            if self.robPos == self.target_box0:
                self.destPub.publish(self.target_box0[0], self.target_box0[1], self.target_box0[2], self.gripperOff)
                self.working = False
                self.gripping = True
                self.get_logger().info("Object sorted")
        if self.working and self.id == 1:
            self.get_logger().info("Sorting Object...")
            self.destPub.publish(self.target_box1[0], self.target_box1[1], self.target_box1[2], self.gripperOn)
            if self.robPos == self.target_box1:
                self.destPub.publish(self.target_box1[0], self.target_box1[1], self.target_box1[2], self.gripperOff)
                self.working = False
                self.gripping = True
                self.get_logger().info("Object sorted")



def main(args=None):
    rclpy.init(args=args)
    move_gripper = MoveGripper()
    rclpy.spin(move_gripper)
    move_gripper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
