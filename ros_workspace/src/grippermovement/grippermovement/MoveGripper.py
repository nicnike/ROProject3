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
        self.thresholdPx = 40
        self.threshold = [0.02, 0.02, 0.02]
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
        self.offsetPos = self.grippingzone + (self.objPos - self.grippingzoneInPx) * 0.025  # /40 / 100 (40px = 1cm zu m)

    def moveGripper(self):
        move = RobotPosWithGripper()
        if not self.init:
            self.get_logger().info("Initialization...")
            self.destPub.publish(-1, -1, -1, False)
            self.init = True
            self.gripping = True
            time.sleep(10)
            self.get_logger().info("Initialization done")

        if self.gripping:
            self.get_logger().info("Moving to GrippingZone")
            move.pos_x = self.grippingzone[0]
            move.pos_y = self.grippingzone[1]
            move.pos_z = self.grippingzone[2]
            move.gripperActivate = self.gripperOff
            self.destPub.publish(move)
        if self.objPos[1] - self.grippingzoneInPx[1] < self.thresholdPx:
            self.get_logger().info("Picking Object...")
            self.calcOffsetPos()
            move.pos_x = self.offsetPos[0]
            move.pos_y = self.offsetPos[1]
            move.pos_z = self.offsetPos[2]
            move.gripperActivate = self.gripperOff
            self.destPub.publish(move)
            time.sleep(2)
            move.pos_x = self.offsetPos[0]
            move.pos_y = self.offsetPos[1]
            move.pos_z = self.down
            move.gripperActivate = self.gripperOn
            self.destPub.publish(move)
            time.sleep(0.2)
            self.gripping = False
            self.working = True
            self.get_logger().info("Object picked")

        if self.working and self.id == 0:
            self.get_logger().info("Sorting Object...")
            move.pos_x = self.target_box0[0]
            move.pos_y = self.target_box0[1]
            move.pos_z = self.target_box0[2]
            move.gripperActivate = self.gripperOn
            self.destPub.publish(move)
            if  self.target_box0 - self.robPos < self.threshold:
                move.pos_x = self.target_box0[0]
                move.pos_y = self.target_box0[1]
                move.pos_z = self.target_box0[2]
                move.gripperActivate = self.gripperOff
                self.destPub.publish(move)
                self.working = False
                self.gripping = True
                self.get_logger().info("Object sorted")
        if self.working and self.id == 1:
            move.pos_x = self.target_box1[0]
            move.pos_y = self.target_box1[1]
            move.pos_z = self.target_box1[2]
            move.gripperActivate = self.gripperOn
            self.destPub.publish(move)
            diff = self.robPos - self.target_box1
            if self.target_box1 - self.robPos < self.threshold:
                move.pos_x = self.target_box1[0]
                move.pos_y = self.target_box1[1]
                move.pos_z = self.target_box1[2]
                move.gripperActivate = self.gripperOff
                self.destPub.publish(move)
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
