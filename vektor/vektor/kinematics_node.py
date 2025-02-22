import rclpy
from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
from my_interfaces.msg import VektorTeleop
from my_interfaces.msg import VektorRPM
from math import pi, sin, cos, radians

# Globals
r = 0.029 # meters
d = 0.075 # meters
V = round(r * (180 * 2 * pi / 60), 2)
W = (2*pi)/2 # rad/s -> one rotation every 2 seconds
rpm_factor = (60 / (2*pi))

class WheelVelocityPublisher(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.publisher_ = self.create_publisher(VektorRPM, 'target_rpm', 2)
        self.get_logger().info(f"Publishing in topic: /target_rpm")
        self.teleop_subscription = self.create_subscription(VektorTeleop, 'teleop', self.teleop_callback, 2) # subscribe to /Joy node

    def teleop_callback(self, msg):
        theta = radians(msg.angle)
        magnitude = msg.magnitude
        wz = msg.wz

        vx = (V*magnitude) * cos(theta)
        vy = (V*magnitude) * sin(theta)
        wz = W * wz

        u1 = round((-d*wz + vx) / r * rpm_factor, 4)
        u2 = round((-d*wz - cos(pi/3)*vx - sin(pi/3)*vy) / r * rpm_factor, 4)
        u3 = round((-d*wz - cos(pi/3)*vx + sin(pi/3)*vy) / r * rpm_factor, 4)

        msg = VektorRPM()
        msg.rpm = [u1,u2,u3]
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocityPublisher()
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
