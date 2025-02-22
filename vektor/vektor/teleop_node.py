import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from my_interfaces.msg import VektorTeleop
from sensor_msgs.msg import Joy
from math import atan2, degrees, radians, sin, cos, pi

#DEAD_ZONE = 0.05

wheel_radius = 0.029 # meters
bot_radius = 0.075 # meters
bot_velocity = wheel_radius * (180 * 2 * pi / 60)

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.declare_parameter('input_mode', 'joy')
        self.input_mode = self.get_parameter('input_mode').value
        self.get_logger().info(f"Input mode selected: {self.input_mode}")

        self.publisher_ = self.create_publisher(VektorTeleop, 'teleop', 2)
        self.get_logger().info(f"Publishing in topic: /teleop")

        if self.input_mode == 'keyboard':
            # Set up a timer to poll keyboard input
            self.keyboard_timer = self.create_timer(0.1, self.keyboard_callback)
            # Configure terminal for non-blocking keyboard input
            self.old_terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else: # if invalid default to joy
            self.get_logger().warning("Invalid input mode. Defaulted to 'joy' ..")
            self.joy_subscription = self.create_subscription(
                Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg: Joy):

        # angle determination
        x = round(-msg.axes[0], 4) #if abs(msg.axes[0]) > DEAD_ZONE else 0.0
        y = round(msg.axes[1], 4) #if abs(msg.axes[1]) > DEAD_ZONE else 0.0

        theta = atan2(y, x) #has built in divide by zero handling
        angle = (360 + degrees(theta)) % 360
        angle = angle - (angle % 10)

        # magnitude determination
        if x==0 and y == 0:
            magnitude = 0
        else:
            # magnitude = (-msg.axes[5] + 1.0)/2 #fine magnitude control
            magnitude = 1 if(msg.axes[4] < 0.5) else 0

        # wz determination
        # wz = msg.axes[3] if abs(msg.axes[3]) > DEAD_ZONE else 0.0
        if msg.axes[3] > 0.4:
            wz = 1
        elif msg.axes[3] < -0.4:
            wz = -1
        else:
            wz = 0

        msg = VektorTeleop()
        msg.angle = int(angle)
        msg.magnitude = magnitude
        msg.wz = wz
        self.publisher_.publish(msg)

    def keyboard_callback(self):
        # Check if there is keyboard input available (non-blocking)
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)

            # match key:
            #     case '1'|'d':
            #         angle = 0 
            #         magnitude = 1
            #         wz = 0
            #     case '2':
            #         angle = 30 
            #         magnitude = 1
            #         wz = 0
            #     case '3'|'w':
            #         angle = 90 
            #         magnitude = 1
            #         wz = 0
            #     case '4':
            #         angle = 150 
            #         magnitude = 1
            #         wz = 0
            #     case '5'|'a':
            #         angle = 180 
            #         magnitude = 1
            #         wz = 0
            #     case '6':
            #         angle = 210 
            #         magnitude = 1
            #         wz = 0
            #     case '7'|'s':
            #         angle = 270 
            #         magnitude = 1
            #         wz = 0
            #     case '8':
            #         angle = 330 
            #         magnitude = 1
            #         wz = 0
            #     case 'q':
            #         angle = 0 
            #         magnitude = 0
            #         wz = 1.0
            #     case 'e':
            #         magnitude = 0
            #         angle = 0 
            #         wz = -1
            #     case _:
            #         print(key)
            #         angle = 0 
            #         magnitude = 0
            #         wz = 0

            match key:
                case '6':
                    angle = 0
                    magnitude = 1
                    wz = 0
                case '9':
                    angle = 30 
                    magnitude = 1
                    wz = 0
                case '8':
                    angle = 90 
                    magnitude = 1
                    wz = 0
                case '7':
                    angle = 150 
                    magnitude = 1
                    wz = 0
                case '4':
                    angle = 180 
                    magnitude = 1
                    wz = 0
                case '1':
                    angle = 210 
                    magnitude = 1
                    wz = 0
                case '2':
                    angle = 270 
                    magnitude = 1
                    wz = 0
                case '3':
                    angle = 330 
                    magnitude = 1
                    wz = 0
                case 'a':
                    angle = 0 
                    magnitude = 0
                    wz = 1
                case 'd':
                    magnitude = 0
                    angle = 0 
                    wz = -1
                case '5':
                    angle = 0 
                    magnitude = 0
                    wz = 0
                case _:
                    # print(key)
                    angle = 0 
                    magnitude = 0
                    wz = 0
            
            msg = VektorTeleop()
            msg.angle = angle
            msg.magnitude = magnitude
            msg.wz = wz
            self.publisher_.publish(msg)

    def destroy_node(self):
        # Restore terminal settings if keyboard mode was used
        if self.input_mode == 'keyboard':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # node.get_logger().info("Keyboard Interrupt, shutting down node.")
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
