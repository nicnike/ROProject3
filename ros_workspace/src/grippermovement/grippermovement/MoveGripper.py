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
        self.target_box0 = [0.0, 0.0, 0.0]
        self.target_box1 = [-0.08, 0.0, 0.0]
        self.grippingzone = [-0.04, -0.085, 0.07]
        self.pickup = [-0.04, -0.085, 0.03]
        self.initMove = [1.0, 1.0, -1.0]
        self.robPos = [0.0, 0.0, 0.0]
        self.offsetPos = [0.0 , 0.0, 0.0]
        self.threshold = [0.02, 0.02, 0.02]

        self.grippingzoneInPx = [-420, 135, 0]
        self.defaultPos = [1000, 1000, 1000]
        self.objPos = self.defaultPos
        self.thresholdPx = 40

        self.off = False
        self.on = True

        self.startTime = time.time()
        self.get_logger().info(str(self.startTime))

        self.timeToReach = 1
        self.timeToPick = 0.8
        self.timeToSort = 2
        self.down = 0.08

        self.id = -1
        self.init = False
        self.gripping = False
        self.working = False
        self.gripperOn = True
        self.gripperOff = False

        self.cat = 0
        self.unicorn = 1


        self.posSub = self.create_subscription(
            RobotPos,
            'robot_position',
            self.callback_rob_pos,
            10)

        self.destPub = self.create_publisher(
            RobotPosWithGripper,
            'robotposwithgripper',
            10)

        self.objPosSub = self.create_subscription(
            ObjectPosition,
            'object_position',
            self.callback_obj_pos,
            10)




    def callback_rob_pos(self, msg):
        """
        this function is called everytime a new message is published on the 'robot_position' topic

        @param msg: is the published message on the 'robot_position' topic
        """
        self.robPos = [msg.pos_x, msg.pos_y, msg.pos_z]
        self.movegripper()
        # self.get_logger().info(str(self.robPos))


    def callback_obj_pos(self, msg):
        """
        this function is called everytime a new message is published on the 'object_position' topic

        @param msg: is the published message on the 'object_information' topic
        """
        self.objPos = [msg.position.y, msg.position.x, msg.position.z]
        # y and x swapped because camera has different axis
        self.id = msg.classification
        self.get_logger().info(str(self.objPos))

    def init_zone_values(self):
        """
        initialize the destination position arrays once
        """
        self.grippingzone = np.add(self.grippingzone, self.robPos)
        self.target_box0 = np.add(self.target_box0, self.robPos)
        self.target_box1 = np.add(self.target_box1, self.robPos)
        self.pickup = np.add(self.pickup, self.robPos)
        self.offsetPos = self.grippingzone

        self.get_logger().info(str(self.grippingzone))

    def calc_object_offset_pos(self):
        """
        calculates the offset between the grippingzone and the actual object position
        """
        #self.offsetPos = np.add(self.grippingzone, np.multiply((np.subtract(self.objPos, self.grippingzoneInPx)), 0.025))  # /40 / 100 (40px = 1cm zu m)
        self.offsetPos[0] = self.grippingzone[0] - 0.0025 * (self.objPos[0] - self.grippingzoneInPx[0])

    def pub_pos(self, moveTo, gripper):
        """
        this function creates an object for the "robotposwithgripper" topic

        @param moveTo: the desired position array
        @param gripper: information whether the gripper should be activated or not
        """
        move = RobotPosWithGripper()
        move.pos_x = moveTo[0]
        move.pos_y = moveTo[1]
        move.pos_z = moveTo[2]
        move.activate_gripper = gripper
        return move

    def initialize(self):
        """
        this function initialize the destination position arrays once
        """
        exec = RobotPosWithGripper()
        self.get_logger().info("Initialization...")
        exec = self.pub_pos(self.initMove, self.gripperOff)
        self.get_logger().info(str(exec))
        self.destPub.publish(exec)

        if time.time() - self.startTime >= 8:
            self.get_logger().info(str(self.robPos))
            self.init_zone_values()
            self.init = True
            self.gripping = True
            self.get_logger().info("Initialization done")

    def movegripper(self):
        """
        this function is called by the ros spin function picks up the objects
        and sends position and gripping data to the controller through a custom interface
        """
        exec = RobotPosWithGripper()
        # self.get_logger().info(str(self.init))
        # self.get_logger().info(str(time.time() - self.startTime))

        if not self.init:
            self.initialize()

        if self.gripping:
            self.get_logger().info("Moving to GrippingZone")
            exec = self.pub_pos(self.grippingzone, self.gripperOff)
            self.destPub.publish(exec)
            self.gripping = False

        if (self.objPos[0] - self.grippingzoneInPx[0] < self.thresholdPx) and not self.working:
            self.get_logger().info("Picking Object...")
            self.calc_object_offset_pos()
            exec = self.pub_pos(self.offsetPos, self.off)
            self.destPub.publish(exec)
            self.get_logger().info("Moving to Object:" + str(exec))
            time.sleep(self.timeToReach)
            exec.pos_z = self.down
            self.get_logger().info("Moving down:" + str(exec))
            exec.activate_gripper = self.gripperOn
            self.destPub.publish(exec)
            time.sleep(self.timeToPick)
            exec = self.pub_pos(self.pickup, self.gripperOn)
            self.destPub.publish(exec)
            self.get_logger().info("Moving Up:" + str(exec))
            time.sleep(self.timeToPick)
            self.gripping = False
            self.working = True
            self.get_logger().info("Object picked")
            self.objPos = self.defaultPos


        if self.working and self.id == self.cat:
            self.get_logger().info("Sorting Object...")
            exec = self.pub_pos(self.target_box0, self.gripperOn)
            self.destPub.publish(exec)
            self.get_logger().info("Moving to Box #1:" + str(exec))
            if  (np.subtract(self.target_box0, self.robPos) < self.threshold).all():
                exec.activate_gripper = self.gripperOff
                self.destPub.publish(exec)
                self.get_logger().info("Releasing Gripper")
                self.working = False
                self.gripping = True
                self.get_logger().info("Object sorted")
        if self.working and self.id == self.unicorn:
            self.get_logger().info("Sorting Object...")
            exec = self.pub_pos(self.target_box1, self.gripperOn)
            self.destPub.publish(exec)
            self.get_logger().info("Moving to Box #2:" + str(exec))
            if (np.subtract(self.target_box1, self.robPos) < self.threshold).all():
                exec.activate_gripper = self.gripperOff
                self.destPub.publish(exec)
                self.get_logger().info("Releasing Gripper")
                time.sleep(self.timeToSort)
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
