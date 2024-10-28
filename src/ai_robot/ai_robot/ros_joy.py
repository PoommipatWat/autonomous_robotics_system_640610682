import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[-1]*0.23*2
        twist.angular.z = msg.axes[-2]*0.05
        self.twist_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    joy_publisher = JoyPublisher()
    rclpy.spin(joy_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()