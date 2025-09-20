import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped


class Relay(Node):

    def __init__(self):
        super().__init__('relay')
        self.drive_subscriber = self.create_subscription(
            AckermannDriveStamped,
            'drive_relay',
            self.listener_callback,
            10) 

    def listener_callback(self, msg):
        self.get_logger().info('I heard v: "%s"' % msg.drive.speed)
        self.get_logger().info('I heard d: "%s"' % msg.drive.steering_angle)


def main(args=None):
    rclpy.init(args=args)
    relay = Relay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()