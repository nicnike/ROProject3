import rclpy
import time
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from custom_interfaces.msg import ObjectPosition, RobotPosWithGripper
import numpy as np

class MoveGripper(Node):
    def __init__(self):
        super().__init__('move_gripper') # type: ignore
        self.target_box0 = [-0.085724999, 0.091122502, 0.0238125]
        self.target_box1 = [-0.008254997, 0.091122502, 0.0238125]
        self.grippingzone = [-0.008254997, 0.193357505, 0.066040002]
        self.pickup = [-0.008254997, 0.193357505, 0.037465001]
        self.initmove = [1, -1, -1]
        self.grippingzoneInPx = [-400, 135, 0]
        self.timeToReach = 2
        self.timeToPick = 0.2
        self.down = -0.1
        self.offsetPos = [0, 0, 0]

        self.robPos = [0, 0, 0]
        self.objPos = [0, 0, 0]
        self.thresholdPx = 40
        self.threshold = [0.02, 0.02, 0.02]
        self.id = -1
        self.init = False
        self.gripping = False
        self.working = False
        self.on = True
        self.off = False
        self.offset = [0, 0, 0]

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.posSub = self.create_subscription(
            RobotPos,
            'robot_position',
            self.callback_rob_pos,
            10)

        # muss vom Controller als Eingang verarbeitet werden
        self.destPub = self.create_publisher(
            RobotPosWithGripper,
            'robotposwithgripper',
            10)

        self.objPosSub = self.create_subscription(
            ObjectPosition,
            'object_position',
            self.callback_obj_pos,
            10)

    def timer_callback(self):
        self.movegripper()

    def callback_rob_pos(self, msg):
        self.robPos = [msg.pos_x, msg.pos_y, msg.pos_z]


    def callback_obj_pos(self, msg):
        self.objPos = [msg.position.x, msg.position.y, msg.position.z]
        self.id = msg.classification

    def init_zones(self):
        self.offset = self.robPos
        self.grippingzone = np.add(self.grippingzone, self.offset)
        self.target_box0 = np.add(self.target_box0, self.offset)
        self.target_box1 = np.add(self.target_box1, self.offset)
        self.pickup = np.add(self.pickup, self.offset)

    def calc_offset_pos(self):
        self.offsetPos = np.add(self.grippingzone, np.multiply((np.subtract(self.objPos, self.grippingzoneInPx)), 0.025))  # /40 / 100 (40px = 1cm zu m)


    def pub_pos(self, moveTo, gripper):

        move.pos_x = moveTo[0]
        move.pos_y = moveTo[1]
        move.pos_z = moveTo[2]
        move.gripperActivate = gripper
        return move

    def movegripper(self):
        move = RobotPosWithGripper()
        if not self.init:
            self.get_logger().info("Initialization...")
            move.pos_x = self.initmove[0]
            move.pos_y = self.initmove[1]
            move.pos_z = self.initmove[2]
            move.gripperActivate = self.off
            self.destPub.publish(move)
            self.init = True
            self.gripping = True
            time.sleep(10)
            self.init_zones()
            self.get_logger().info("Initialization done")

        if self.gripping:
            self.get_logger().info("Moving to GrippingZone")
            move.pub_pos(self.grippingzone, False)
            self.destPub.publish(move)
        if self.objPos[1] - self.grippingzoneInPx[1] < self.thresholdPx:
            self.get_logger().info("Picking Object...")
            self.calc_offset_pos()
            move.pos_x = self.offsetPos[0]
            move.pos_y = self.offsetPos[1]
            move.pos_z = self.offsetPos[2]
            move.gripperActivate = self.off
            self.destPub.publish(move)
            time.sleep(self.timeToReach)
            move.pos_x = self.offsetPos[0]
            move.pos_y = self.offsetPos[1]
            move.pos_z = self.down
            move.gripperActivate = self.on
            self.destPub.publish(move)
            time.sleep(self.timeToPick)
            self.gripping = False
            self.working = True
            self.get_logger().info("Object picked")

        if self.working and self.id == 0:
            self.get_logger().info("Sorting Object...")
            move.pos_x = self.target_box0[0]
            move.pos_y = self.target_box0[1]
            move.pos_z = self.target_box0[2]
            move.gripperActivate = self.on
            self.destPub.publish(move)
            if  np.subtract(self.target_box0, self.robPos) < self.threshold:
                move.pos_x = self.target_box0[0]
                move.pos_y = self.target_box0[1]
                move.pos_z = self.target_box0[2]
                move.gripperActivate = self.off
                self.destPub.publish(move)
                self.working = False
                self.gripping = True
                self.get_logger().info("Object sorted")
        if self.working and self.id == 1:
            move.pos_x = self.target_box1[0]
            move.pos_y = self.target_box1[1]
            move.pos_z = self.target_box1[2]
            move.gripperActivate = self.on
            self.destPub.publish(move)
            diff = self.robPos - self.target_box1
            if np.subtract(self.target_box1, self.robPos) < self.threshold:
                move.pos_x = self.target_box1[0]
                move.pos_y = self.target_box1[1]
                move.pos_z = self.target_box1[2]
                move.gripperActivate = self.off
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
