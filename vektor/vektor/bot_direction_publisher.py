import math
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node

DEAD_ZONE = 0.05

class BotDirectionPublisher(Node):
    def __init__(self):
        super().__init__('bot_direction_publisher') # node name

        self.publisher = self.create_publisher(
            msg_type=String, topic='/bot_direction', qos_profile=10
        ) # data of type string, publish to topic /bot_direction, qos_profile read here https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-profiles

        self.subscription = self.create_subscription(
            msg_type=Joy, topic='/joy', callback=self.joy_callback, qos_profile=10
        ) # subscribe to /Joy node

        self.get_logger().info("Bot Direction Node initialized. listening to joystick input...")

    def joy_callback(self, msg):
        """0, 1, 3 are the index of joy sticks in message published by /joy"""
        strafe_x = msg.axes[0] if abs(msg.axes[0]) > DEAD_ZONE else 0
        strafe_y = msg.axes[1] if abs(msg.axes[1]) > DEAD_ZONE else 0
        rotational_omega = -msg.axes[3] if abs(msg.axes[3]) > DEAD_ZONE else 0
        
        if rotational_omega > 0:
            wz = 1
        elif rotational_omega < 0:
            wz = -1
        else:
            wz = 0

        if strafe_x == 0 and strafe_y == 0: 
            angle = 501
        else:
            # Calculate the angle in degrees
            angle = (2*math.pi + math.atan2(strafe_y, strafe_x)) % 360 #(0 to 2pi)
            angle = angle - (angle%10) #(0 to 2pi, only multiples of 10)

        
        message_to_be_published = f"{angle},{wz}"
        
        self.publish_message(message_to_be_published)

    def publish_message(self, msg):
        message = String()
        message.data = msg
        self.publisher.publish(message)
        self.get_logger().info(f"Published: {message.data}") # log message to console


def main(args=None):
    rclpy.init(args=args)
    node = BotDirectionPublisher()
    try:
        rclpy.spin(node=node) # keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()