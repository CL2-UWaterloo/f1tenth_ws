import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 1000) # Publish at 1000 Hz
        self.timer = self.create_timer(0.001, self.timer_callback)

        self.declare_parameter('v', 0)
        self.declare_parameter('d', 0)

    def timer_callback(self):
        msg = AckermannDriveStamped()
        v = self.get_parameter('v').value
        d = self.get_parameter('d').value
        msg.drive.speed = v
        msg.drive.steering_angle = d
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    talker = Talker()

    rclpy.spin(talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
