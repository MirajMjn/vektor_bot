import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from vektor.pid_controller import PIDController
from vektor.motor import Motor
from wheel_sensors_interfaces.msg import MotorSpeeds, WheelRPMs, BotDirections, WheelDistances
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
# import RPi.GPIO as GPIO


KP, KI, KD = 0.9, 0.6857, 0.0


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_contorl_node')
        self.target_rpms_subscriber = self.create_subscription(MotorSpeeds, '/bot/target_rpms', self.target_rpms_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.current_rpms_subscriber = self.create_subscription(WheelRPMs, '/bot/current_rpms', self.current_rpms_callback, 10)
        self.bot_state_subscriber = self.create_subscription(Bool, '/l1_pressed', self.bot_state_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.angle_subscriber = self.create_subscription(BotDirections, '/bot_directions', self.angle_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        # self.distance_subscriber = self.create_subscription(WheelDistances, '/bot/tracking_wheel_pulses', self.distance_callback, 10)
        # self.create_timer(0.3, callback=self.run_motors)
        self.get_logger().info('Motor Controller Node Initialized')

        self.l1_pressed = False
        self.angle = self.wbz = self.magnitude = 0
        self.u1 = self.u2 = self.u3 = 0
        self.rpm1 = self.rpm2 = self.rpm3 = 0
        self.pulse_left = self.pulse_right = self.pulse_back = 0
        self.old_log_msg = ''

        self.motors = [
            Motor(6, 5, 'motor1'), # physical 29, 31
            Motor(17, 27, 'motor2'), # physical 11, 13
            Motor(24, 23, 'motor3') # physical 16, 18
        ]

        for motor in self.motors:
            motor.pid_controller = PIDController(KP, KI, KD)

    def angle_callback(self, msg):
        self.angle, self.wbz, self.magnitude = msg.theta, msg.wbz, msg.magnitude

    def bot_state_callback(self,msg):
        self.l1_pressed = msg.data # true or false
        if not self.l1_pressed:
            self.stop_all_motors()
        
    # def target_rpms_callback(self, msg):
    #     self.u1, self.u2, self.u3 = msg.u1, msg.u2, msg.u3
    #     if all(u == 0 for u in [self.u1, self.u2, self.u3]):
    #         self.stop_all_motors()

    def target_rpms_callback(self, msg):
        self.u1, self.u2, self.u3 = msg.u1, msg.u2, msg.u3
        if all(u == 0 for u in [self.u1, self.u2, self.u3]):
            self.stop_all_motors()
        else:
            self.run_motors()  # Run motors immediately on new command

    def current_rpms_callback(self, msg):
        self.rpm1, self.rpm2, self.rpm3 = msg.rpm1, msg.rpm2, msg.rpm3
        self.run_motors()

    def distance_callback(self, msg):
        self.pulse_left = msg.dead_wheel_left
        self.pulse_right = msg.dead_wheel_right
        self.pulse_back = msg.dead_wheel_back

    def run_motors(self):  # Todo: 
        if self.l1_pressed and (self.magnitude or self.wbz) and any([self.u1, self.u2, self.u3]):
            # read about zip here: https://www.geeksforgeeks.org/zip-in-python/
            for motor, u, rpm in zip(self.motors, [self.u1, self.u2, self.u3], [self.rpm1, self.rpm2, self.rpm3]):
                motor.rotate(u, rpm)

        new_log_message = (
            f"\n"
            f"ang: {self.angle:<10}wbz: {self.wbz:<10}magntud: {self.magnitude:.2f}\n"
            f"u1: {self.u1:<10}u2: {self.u2:<10}u3: {self.u3}\n"
            f"r1: {self.rpm1:<10}r2: {self.rpm2:<10}r3: {self.rpm3}\n"
            # f"dL: {self.pulse_left:<10}dR: {self.pulse_right:<10}dB: {self.pulse_back}\n"
            f"{'-'*50}\n"
        )
        if self.old_log_msg != new_log_message:
            self.get_logger().info(new_log_message)
            self.old_log_msg = new_log_message
        
    def stop_all_motors(self):
        for motor in self.motors:
            motor.stop()

    def __del__(self):  # destructor for motor class, How does it work?
        for motor in self.motors:
            motor.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # recorder.print_record()
        # recorder.print_record(KP, KI, KD)
        # GPIO.cleanup()
        # if rclpy.ok():  #Prevent multiple shutdown calls
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()