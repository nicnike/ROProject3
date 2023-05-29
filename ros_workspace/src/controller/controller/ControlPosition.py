import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd


class ControlPosition(Node):
    def __init__(self):
        super().__init__('control_position') # type: ignore
        #Will be sent later to the controller
        self.posSub = self.create_subscription(
            RobotPos,
            'position_publisher',
            self.position_callback,
            10)
        
        self.velPub = self.create_publisher(
            RobotCmd,
            'command_subscriber',
            10)
        
        self.posSub = self.create_subscription(
            RobotPos,
            'robot_reference_position',
            self.desired_position_callback,
            10)
        
        self.current_position = RobotPos()
        self.target_position = RobotPos()

    def position_callback(self, data):
        self.current_position = data

    def desired_position_callback(self, data):
        self.target_position = data
        self.publishVelocity()

    def publishVelocity(self):
        velocity = self.calculate_control_signal()
        self.velPub.publish(velocity)

    def calculate_control_signal(self):
        kx = 1
        ky = 1
        kz = 1
        robot_cmd = RobotCmd()
        robot_cmd.vel_x = kx * (self.target_position.pos_x - self.current_position.pos_x)
        robot_cmd.vel_y = ky * (self.target_position.pos_y - self.current_position.pos_y)
        robot_cmd.vel_z = kz * (self.target_position.pos_z - self.current_position.pos_z)
        robot_cmd.activate_gripper = False
        return robot_cmd

def main(args=None):
    rclpy.init(args=args)
    control_position = ControlPosition()
    rclpy.spin(control_position)
    control_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
