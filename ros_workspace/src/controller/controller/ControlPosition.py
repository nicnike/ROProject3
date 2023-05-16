import rclpy
from rclpy.node import Node
import ro45_portalrobot_interfaces.msg as msg


class ControlPosition(Node):
    def __init__(self):
        super().__init__('control_position') # type: ignore
        #Will be sent later to the controller
        self.target_value = [0,0,0]
        self.posSub = self.create_subscription(
            msg.RobotPos,
            'position_publisher',
            self.position_callback,
            10)
        
        self.velPub = self.create_publisher(
            msg.RobotCmd,
            'command_subscriber',
            10)
        
        #Logic receive target values!
        #self.posSub = self.create_subscription(
        #    msg.RobotPos,
        #    'placeholder1',
        #    self.position_callback,
        #    10)

        self.kp = 5

    def position_callback(self, data):
        regelabweichung = [self.target_value[i] - data.position[i] for i in range(3)]

        # If the control deviation is less than 0.1, it is set to 0
        for i in range(len(regelabweichung)):
            if regelabweichung[i] < 0.1:
                regelabweichung[i] = 0
        
        # Control deviation is multiplied by Kp
        regelabweichung = [self.kp * regelabweichung[0],self.kp * regelabweichung[1],self.kp * regelabweichung[2]]

        # Publish the control deviation as a velocity
        self.velPub.publish(msg.RobotCmd(x=regelabweichung[0],y=regelabweichung[1],z=regelabweichung[2],gripper=False))
        self.get_logger().info('Publishing: "%s"' % regelabweichung)

def main(args=None):
    rclpy.init(args=args)
    control_position = ControlPosition()
    rclpy.spin(control_position)
    control_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
