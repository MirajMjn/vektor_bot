import rclpy
from rclpy.node import Node
from my_interfaces import VektorPID
import RPi.GPIO as GPIO

from DCMotor import DCMotor

class GPIOInterfaceNode(Node):
    def __init__(self):
        super().__init__('GPIO_interface_node')

        self.declare_parameter('subscription', 'PID_output')
        self.declare_parameter('in1', 5)
        self.declare_parameter('in2', 6)
        self.declare_parameter('id', 1)

        self.subscription = self.get_parameter('subscription').value
        self.id = self.get_parameter('id').value
        self.in1 = self.get_parameter('in1').value
        self.in2 = self.get_parameter('in2').value

        self.motor = DCMotor(in1=self.in1, in2=self.in2, id=self.id)

        self.pwm = 0.0
        self.direction = 1

        self.PWM_subscription = self.create_subscription(VektorPID, self.subscription, self.PID_callback, 2)
        
        # Set up a Motor Control Timer
        self.run_motor_timer = self.create_timer(0.1, self.run_motor)
        
    def PID_callback(self, msg):
        self.direction = msg.direction
        self.pwm = msg.pwm

    def run_motors(self):
        if self.direction == 1.0:
            self.motor.forward(self.pwm)
        else:
            self.motor.backward(self.pwm)

    def destroy_node(self):
        GPIO.cleanup([self.in1, self.in2]) # -> cleans up pins specific to this node instance
        super().destroy_node()        

def main(args=None):
    rclpy.init(args=args)
    node = GPIOInterfaceNode()
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
