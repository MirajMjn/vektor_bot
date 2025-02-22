import rclpy
from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
from my_interfaces.msg import VektorRPM
from my_interfaces.msg import VektorPID

class PIDControlNode(Node):
    def __init__(self):
        super().__init__('PID_control_node')
        
        self.publisher_ = self.create_publisher(VektorPID, 'PID_output', 2)
        self.get_logger().info(f"PID_motor_controller_node publishing in topic: /PID_output")

        self.target_subscription = self.create_subscription(VektorRPM, 'target_rpm', self.target_callback, 2)
        self.serial_subscription = self.create_subscription(VektorRPM, 'serial_data', self.serial_callback, 2)

        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)
        self.declare_parameter('id', 1)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.id = self.get_parameter('id').value

        self.target_rpm = 0.0
        self.current_rpm = 0.0
        self.error = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.update_rate = 0.4
        self.derivative_threshold = 5.0

        self.min_output = 0.0
        self.max_output = 70.0
        self.saturated = True

        self.direction = 1
        self.output = 0.0

        # Set up a PID Timer
        self.PID_timer = self.create_timer(self.update_rate, self.PID_controller)
    
    def target_callback(self, msg):
        try:
            self.target_rpm = msg.rpm[self.id-1]
            self.direction = 1 if(self.target_rpm >= 0) else -1
            self.target_rpm = abs(self.target_rpm)
        except Exception as e:
            self.get_logger().error(f"[Exception] : {e} : PID_controller_{self.id} failed to extract target_rpm")
            # self.target_rpm = None
    
    def serial_callback(self, msg):
        try:
            self.current_rpm = msg.rpm[self.id-1]
        except Exception as e:
            self.get_logger().error(f"[Exception] : {e} : PID_controller_{self.id} failed to extract current_rpm")
            # self.current_rpm = None

    def PID_controller(self):
        try:
            self.error = self.target_rpm - self.current_rpm
            del_e = self.error - self.last_error

            self.derivative = del_e / self.update_rate
            self.last_error = self.error

            # Prevent integral wind-up
            if(del_e >= -self.derivative_threshold and del_e <= self.derivative_threshold and self.saturated == False):
                self.integral += self.error * self.update_rate
            if(self.Ki!=0):
                self.integral = max(min(self.integral, self.max_output / self.Ki), -self.max_output / self.Ki)
            
            
            self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative

            self.output = max(self.min_output, min(self.max_output, self.output))

            if(self.output == self.min_output or self.output == self.max_output):
                self.saturated = True
            else:
                self.saturated = False

            msg = VektorPID()
            msg.direction = self.direction
            msg.pwm = self.output
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[Exception] : {e} encountered in PID_controller function")
        
def main(args=None):
    rclpy.init(args=args)
    node = PIDControlNode()
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
