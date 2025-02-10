import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from math import pi, sin, cos

# Globals
r = 0.029 # meters
d = 0.075 # meters
# V = round(r * (180 * 2 * pi / 60), 2)
V = 0.5 # m/s
W = (2*pi)/2 # rad/s
rpm_factor = (60 / (2*pi))

class WheelVelocityPublisher(Node):
    def __init__(self):
        super().__init__('wheel_velocities')
        print("kinematics_node publishing to topic /wheel_velocities")
        self.publisher_ = self.create_publisher(Float32MultiArray, 'wheel_velocities', 10)
        self.twist_subscription = self.create_subscription(TwistStamped, 'bot_twist', self.twist_callback, 10) # subscribe to /Joy node

    def twist_callback(self, msg):

        wz = round(msg.twist.angular.z * W, 2)

        vx = round(msg.twist.linear.x * V, 2)
        vy = round(msg.twist.linear.y * V, 2)

        u1 = round((-d*wz + vx) / r * rpm_factor, 2)# how fast wheel 1 must rotate
        u2 = round((-d*wz - cos(pi/3)*vx - sin(pi/3)*vy) / r * rpm_factor, 2)
        u3 = round((-d*wz - cos(pi/3)*vx + sin(pi/3)*vy) / r * rpm_factor, 2)

        # Create a Float32MultiArray message
        msg = Float32MultiArray()
        # Populate the data field with the 3 float values
        msg.data = [u1, u2, u3]

        # Publish the message
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
