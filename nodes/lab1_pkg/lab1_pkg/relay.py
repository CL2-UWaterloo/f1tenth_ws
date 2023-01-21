import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class Relay(Node):

    def __init__(self):
        super().__init__('relay')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 1000) # Publish at 1000 Hz
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        msg.drive.speed *= 3.0
        msg.drive.steering_angle *= 3.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    relay = Relay()

    rclpy.spin(relay)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
