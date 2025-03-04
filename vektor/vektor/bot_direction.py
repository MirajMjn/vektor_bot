import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from wheel_sensors_interfaces.msg import BotDirections

DEAD_ZONE = 0.3

def get_angle(strafe_x, strafe_y):
    """Compute movement angle, snapped to 30-degree increments."""
    if abs(strafe_x) <= DEAD_ZONE and abs(strafe_y) <= DEAD_ZONE:
        return 501.0  # Special case for stopping
    angle = (360 + math.degrees(math.atan2(strafe_y, strafe_x))) % 360 *1.0
    return angle - (angle % 5)  # Round to nearest 30

def get_magnitude(strafe_x, strafe_y):
    """Compute movement magnitude (speed)."""
    return 0.0 if abs(strafe_x) <= DEAD_ZONE and abs(strafe_y) <= DEAD_ZONE else 1.0

def get_wbz(rotate_x):
    """Determine rotation speed (wz) with a dead zone."""
    return 0 if int(abs(rotate_x)) < 1 else int(rotate_x)

class BotDirectionsNode(Node):
    def __init__(self):
        super().__init__('bot_directions_node')  # Node name
        self.publisher = self.create_publisher(BotDirections, '/bot_directions', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("Bot Direction Node initialized. Listening to joystick input...")
        self.old_log_msg = ''

    def joy_callback(self, msg):
        """Convert joystick input to movement direction and publish it."""
        strafe_x, strafe_y, rotate_x = -msg.axes[0], msg.axes[1], msg.axes[3]

        bot_direction_msg = BotDirections()
        bot_direction_msg.theta = get_angle(strafe_x, strafe_y)
        bot_direction_msg.wbz = get_wbz(rotate_x)
        bot_direction_msg.magnitude = get_magnitude(strafe_x, strafe_y)
        self.publisher.publish(bot_direction_msg)

        new_log_msg=f"theta={bot_direction_msg.theta}, wbz={bot_direction_msg.wbz}, magnitude={bot_direction_msg.magnitude:.2f}"
        if new_log_msg != self.old_log_msg:
            self.get_logger().info(new_log_msg)
            self.old_log_msg = new_log_msg

def main(args=None):
    rclpy.init(args=args)
    node = BotDirectionsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
