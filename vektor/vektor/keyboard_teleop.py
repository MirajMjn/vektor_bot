import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from wheel_sensors_interfaces.msg import BotDirections

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop_node")
        self.publisher = self.create_publisher(BotDirections, '/bot_directions', 10)
        self.get_logger().info("keyboard_teleop_node initialized...")
        self.get_logger().info("Press 'w' to move forward, 's' to move backward, 'a' to move left, 'd' to move right, 'q' to stop")
        self.get_logger().info("Press 'space' to exit")
        self.get_logger().info("Press 'q' to rotate left, 'e' to rotate right")

        self.keyboard_timer = self.create_timer(0.1, self.keyboard_callback)
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def keyboard_callback(self):
        """Handles keyboard input"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):  # check if a key is pressed
            key = sys.stdin.read(1)  # Read a single character from standard input
            
            match key:
                case 'w':  # Forward
                    angle = 90
                    magnitude = 1
                    wz = 0
                case 's':  # Backward
                    angle = 270
                    magnitude = 1
                    wz = 0
                case 'a':  # Left
                    angle = 180
                    magnitude = 1
                    wz = 0
                case 'd':  # Right
                    angle = 0
                    magnitude = 1
                    wz = 0
                case 'q':  # Rotate Left
                    angle = 0
                    magnitude = 0
                    wz = 1
                case 'e':  # Rotate Right
                    angle = 0
                    magnitude = 0
                    wz = -1
                case ' ':  # Exit
                    self.get_logger().info("Exiting...")
                    rclpy.shutdown()
                    return
                case _:  # Ignore other keys
                    return

            # Create and publish message
            msg = BotDirections()
            msg.angle = angle
            msg.magnitude = magnitude
            msg.wz = wz
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
