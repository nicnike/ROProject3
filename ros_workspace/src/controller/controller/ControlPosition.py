import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from custom_interfaces.msg import RobotPosWithGripper


class ControlPosition(Node):
    def __init__(self):
        super().__init__('control_position') # type: ignore
        #Will be sent later to the controller
        self.currentPosSub = self.create_subscription(
            RobotPos,
            'robot_position',
            self.position_callback,
            10)
        
        self.velPub = self.create_publisher(
            RobotCmd,
            'robot_command',
            10)
        
        self.desiredPosSub = self.create_subscription(
            RobotPosWithGripper,
            'robotposwithgripper',
            self.desired_position_callback,
            10)
        
        self.current_position = RobotPos()
        self.target_position = RobotPosWithGripper()

    def position_callback(self, data):
        self.current_position = data
        self.publishVelocity()

    def desired_position_callback(self, data):
        self.target_position = data

    def publishVelocity(self):
        velocity = self.calculate_control_signal()
        self.velPub.publish(velocity)

    def calculate_control_signal(self):
        kx = 2
        ky = 2
        kz = 2
        robot_cmd = RobotCmd()

        x_diff = self.target_position.pos_x - self.current_position.pos_x
        if x_diff < 0.0005 and x_diff > -0.0005:
             x_diff = 0.0
        y_diff = self.target_position.pos_y - self.current_position.pos_y
        if y_diff < 0.0005 and y_diff > -0.0005:
            y_diff = 0.0
        z_diff = self.target_position.pos_z - self.current_position.pos_z
        if z_diff < 0.0005 and z_diff > -0.0005:
            z_diff = 0.0

        robot_cmd.vel_x = kx * x_diff
        robot_cmd.vel_y = ky * y_diff
        robot_cmd.vel_z = kz * z_diff


        # if robot_cmd.vel_x < 0.004 and robot_cmd.vel_x > -0.004:
        #      robot_cmd.vel_x = 0.0
        # if robot_cmd.vel_y < 0.004 and robot_cmd.vel_y > -0.004:
        #      robot_cmd.vel_y = 0.0
        # if robot_cmd.vel_z < 0.004 and robot_cmd.vel_z > -0.004:
        #      robot_cmd.vel_z = 0.0


        robot_cmd.activate_gripper = self.target_position.activate_gripper
        return robot_cmd

def main(args=None):
    rclpy.init(args=args)
    control_position = ControlPosition()
    rclpy.spin(control_position)
    control_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
