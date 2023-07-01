import rclpy
import time
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from custom_interfaces.msg import ObjectPosition, RobotPosWithGripper
import numpy as np

class MoveGripper(Node):
    """
      MoveGripper subscribes to the "robot_position" topic to receive robot position data and to the
      "object_information" to receive object information data, processes them and publishes
      position and gripping data to "robotposwithgripper" topic.
      """

    def __init__(self):
        """
            Constructor for MoveGripper class.
        """
        super().__init__('move_gripper') # type: ignore
        self.target_box_0 = [-0.01, 0.0, 0.0]
        self.target_box_1 = [-0.08, 0.0, 0.0]
        self.grippingzone = [-0.04, -0.082, 0.065]
        self.pickup = [-0.04, -0.085, 0.03]
        self.init_move = [1.0, 1.0, -1.0]
        self.rob_pos = [0.0, 0.0, 0.0]
        self.offset_pos = [0.0 , 0.0, 0.0]
        self.threshold = [0.02, 0.02, 0.02]

        self.grippingzone_in_px = [-552, 160, 0]
        self.default_pos = [1000, 1000, 1000]
        self.obj_pos = self.default_pos
        self.threshold_in_px = 200

        self.start_time = time.time()
        self.get_logger().info(str(self.start_time))

        self.time_to_reach = 0.8
        self.time_to_pick = 0.4
        self.down = 0.085

        self.id = -1
        self.init = False
        self.gripping = False
        self.working = False
        self.gripperOn = True
        self.gripperOff = False

        self.cat = 0
        self.unicorn = 1


        self.pos_sub = self.create_subscription(
            RobotPos,
            'robot_position',
            self.callback_rob_pos,
            10)

        self.dest_pub = self.create_publisher(
            RobotPosWithGripper,
            'robotposwithgripper',
            10)

        self.obj_pos_sub = self.create_subscription(
            ObjectPosition,
            'object_position',
            self.callback_obj_pos,
            10)




    def callback_rob_pos(self, msg):
        """
        this function is called everytime a new message is published on the 'robot_position' topic

        @param msg: is the published message on the 'robot_position' topic
        """
        self.rob_pos = [msg.pos_x, msg.pos_y, msg.pos_z]
        self.movegripper()
        # self.get_logger().info(str(self.robPos))


    def callback_obj_pos(self, msg):
        """
        this function is called everytime a new message is published on the 'object_position' topic

        @param msg: is the published message on the 'object_information' topic
        """
        self.obj_pos = [msg.position.y, msg.position.x, msg.position.z]
        # y and x swapped because camera has different axis orientation
        self.id = msg.classification
        self.get_logger().info(str(self.obj_pos))

    def init_zone_values(self):
        """
        initialize the destination position arrays once
        """
        self.grippingzone = np.add(self.grippingzone, self.rob_pos)
        self.target_box_0 = np.add(self.target_box_0, self.rob_pos)
        self.target_box_1 = np.add(self.target_box_1, self.rob_pos)
        self.pickup = np.add(self.pickup, self.rob_pos)
        self.offset_pos =  self.grippingzone

        self.get_logger().info("Init!!!")

    def calc_object_offset_pos(self):
        """
        calculates the offset between the grippingzone and the actual object position
        """
        self.get_logger().info("Gripping Zone:" + str(self.grippingzone))
        self.offset_pos[0] = self.grippingzone[0] + 0.00025 * (self.obj_pos[0] - self.grippingzone_in_px[0])
        self.offset_pos[1] = self.grippingzone[1] + 0.00025 * (self.obj_pos[1] - self.grippingzone_in_px[1])
        self.get_logger().info("Gripping Zone:" + str(self.grippingzone))
        self.get_logger().info("Offset Position:" + str(self.offset_pos))


    def pub_pos(self, move_to, gripper):
        """
        this function creates an object for the "robotposwithgripper" topic

        @param move_to: the desired position array
        @param gripper: information whether the gripper should be activated or not
        """
        move = RobotPosWithGripper()
        move.pos_x = move_to[0]
        move.pos_y = move_to[1]
        move.pos_z = move_to[2]
        move.activate_gripper = gripper
        return move

    def initialize(self):
        """
        this function initialize the destination position arrays once
        """
        self.get_logger().info("Initialization...")
        exec = self.pub_pos(self.init_move, self.gripperOff)
        self.get_logger().info(str(exec))
        self.dest_pub.publish(exec)

        if time.time() - self.start_time >= 11:
            self.get_logger().info(str(self.rob_pos))
            self.init_zone_values()
            self.init = True
            self.gripping = True
            self.get_logger().info("Initialization done")

    def movegripper(self):
        """
        this function implements a state machine and updates at 10Hz picks up the objects
        and sends position and gripping data to the controller through a custom interface
        """

        if not self.init:
            self.initialize()

        if self.gripping:
            self.get_logger().info("Moving to GrippingZone")
            self.get_logger().info("Gripping Zone:" + str(self.grippingzone))
            exec = self.pub_pos(self.grippingzone, self.gripperOff)
            self.dest_pub.publish(exec)
            self.gripping = False

        if (self.obj_pos[0] - self.grippingzone_in_px[0] < self.threshold_in_px) and not self.working:
            self.get_logger().info("Picking Object...")
            self.get_logger().info("Gripping Zone:" + str(self.grippingzone))
            self.calc_object_offset_pos()
            exec = self.pub_pos(self.offset_pos, self.gripperOff)
            self.dest_pub.publish(exec)
            self.get_logger().info("Moving to Object:" + str(exec))
            self.get_logger().info("Gripping Zone:" + str(self.grippingzone))
            time.sleep(self.time_to_reach)
            exec.pos_z = self.down
            self.get_logger().info("Moving down:" + str(exec))
            exec.activate_gripper = self.gripperOn
            self.dest_pub.publish(exec)
            time.sleep(self.time_to_pick)
            exec = self.pub_pos(self.pickup, self.gripperOn)
            self.dest_pub.publish(exec)
            self.get_logger().info("Moving Up:" + str(exec))
            time.sleep(self.time_to_pick)
            self.gripping = False
            self.working = True
            self.get_logger().info("Object picked")
            self.obj_pos = self.default_pos


        if self.working and self.id == self.cat:
            self.get_logger().info("Sorting to Box #1...")
            exec = self.pub_pos(self.target_box_0, self.gripperOn)
            self.dest_pub.publish(exec)
            if (np.subtract(self.target_box_0, self.rob_pos) < self.threshold).all():
                exec.activate_gripper = self.gripperOff
                self.dest_pub.publish(exec)
                self.get_logger().info("Releasing Gripper")
                self.working = False
                self.gripping = True
                self.get_logger().info("Object sorted")

        if self.working and self.id == self.unicorn:
            self.get_logger().info("Sorting to Box #2")
            exec = self.pub_pos(self.target_box_1, self.gripperOn)
            self.dest_pub.publish(exec)
            if (np.subtract(self.target_box_1, self.rob_pos) < self.threshold).all():
                exec.activate_gripper = self.gripperOff
                self.dest_pub.publish(exec)
                self.get_logger().info("Releasing Gripper")
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
