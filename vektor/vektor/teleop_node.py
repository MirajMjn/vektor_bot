import rclpy
from rclpy.node import Node
# from vektor.msg import ExtendedTwist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from math import atan2, pi, sin, cos, sqrt, degrees, radians

DEAD_ZONE = 0.05

class TwistStampedPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        print("teleop_node publishing to topic /bot_twist")
        # Create a publisher that will publish Twist messages on the 'cmd_vel' topic
        self.publisher_= self.create_publisher(TwistStamped, 'bot_twist', 10)

        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10) # subscribe to /Joy node

    def joy_callback(self, msg):

        x = -msg.axes[0] if abs(msg.axes[0]) > DEAD_ZONE else 0.0
        y = msg.axes[1] if abs(msg.axes[1]) > DEAD_ZONE else 0.0

        wz = msg.axes[3] if abs(msg.axes[3]) > DEAD_ZONE else 0.0

        theta = atan2(y, x)
        angle = (360 + degrees(theta)) % 360
        angle = angle - angle%5
        theta = radians(angle)

        magnitude = sqrt(x**2 + y**2) / sqrt(2)

        # Create a Twist message
        msg = TwistStamped()

        # Populate header with current time and frame id
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = 'base_link'  # or your appropriate frame

        # Populate Twist part
        msg.twist.linear.x = round(magnitude * cos(theta), 3)
        msg.twist.linear.y = round(magnitude * sin(theta), 3)
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = round(wz, 3)

        # Publish the message
        self.publisher_.publish(msg)
        print(angle)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()