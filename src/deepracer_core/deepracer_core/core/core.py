import rclpy
from rclpy.node import Node
from rclpy.node import QoSProfile
from std_msgs.msg import String
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from geometry_msgs.msg import Twist, Vector3

class Core(Node):

    def __init__(self):

        super().__init__('core_node')
        qos_profile = QoSProfile(depth =10)

        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.sub_msg,
            qos_profile
        )

        self.action_publisher = self.create_publisher(
            ServoCtrlMsg,
            'servo_msg',
            qos_profile
        )
    def action_publish(self, target_steer, target_speed):
        #Function publishes the action and sends it to servo.
        #Args:
        #    target_steer (float): Angle value to be published to servo.
        #    target_speed (float): Throttle value to be published to servo.
        
        result = ServoCtrlMsg()
        result.angle, result.throttle = target_steer, target_speed
        self.get_logger().info(f"Publishing to servo: Steering {target_steer} | Throttle {target_speed}")
        self.action_publisher.publish(result)

    def sub_msg(self, msg):

        try:
            self.lin_vel = msg.linear.x
            self.ang_vel = msg.angular.z
            target_steer, target_speed = self.twist2Servo()
            self.get_logger().info('Subscribed Linear Velocity: {0}, Angular Velocity: {1}'.format(self.lin_vel, self.ang_vel))
            self.action_publish(target_steer, target_speed)

        except Exception as ex:
            self.get_logger().error(f"Failed to publish action: {ex}")
            self.action_publish(0.0, 0.0)


    def twist2Servo(self):
        # steer [-1.0 , 1.0]
        # speed [-1.0 , 1.0]
        target_steer = self.ang_vel
        target_speed = self.lin_vel
        return target_steer, target_speed


def main(args = None):
        rclpy.init(args=args)
        node = Core()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            

if __name__ == '__main__':
    main()
