import RPi.GPIO as GPIO # https://pypi.org/project/RPi.GPIO/
GPIO.setmode(GPIO.BCM)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from vektor.kinematics import target_wheel_rpm
from vektor.motor import DCMotor
from time import sleep

class BotDirectionSubscriber(Node):
    def __init__(self):
        super().__init__('bot_direction_subscriber') # name of node
        print("subscriber initialized")
        self.subscription = self.create_subscription(
            msg_type=String, topic='/bot_direction', callback=self.bot_direction_callback_callback, qos_profile=10
        ) 
        
        self.motor1 = DCMotor(5, 6, 'motor_1') # physical 29, 31 
        self.motor2 = DCMotor(27, 17, 'motor_2')  # physical 11, 13
        self.motor3 = DCMotor(23, 24, 'motor_3') # physical 16 18

        self.ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=0)
        self.current_rpm = [0, 0, 0]
        self.target_rpm = [0, 0, 0]

        # Update motors every 0.1 sec (10 Hz)
        self.create_timer(0.1, self.drive_motors)

        rclpy.get_default_context().on_shutdown(self.cleanup) 

    def bot_direction_callback(self, msg):

        """
        The following is executed in response to joystick messages
        """

        # decoding the joystic message into heading and yaw
        try:
            data = msg.data.split(',')
            theta = float(data[0])
            wbz = float(data[-1])
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Malformed joystick data: {msg.data}")
            return
        
        if theta == 501: # basically default position / no input
            self.target_rpm = [0, 0, 0]
        else:
            # calling the kineamtics to determing u_n based on theta,wz
            self.target_rpm = target_wheel_rpm(theta, wbz)

    def drive_motors(self): #this is called repeatedly on a timer
        try:
            rpms_pico = self.ser.readline().decode().strip()
            if rpms_pico:
                self.current_rpm = [float(rpm) for rpm in rpms_pico.split(',')]
        except (ValueError, IndexError):
            print("Invalid RPM data received. Skipping update.")
            return
        
        # Run PID and apply PWM updates
        self.motor1.rotate(self.target_rpm[0], self.current_rpm[0])
        self.motor2.rotate(self.target_rpm[1], self.current_rpm[1])
        self.motor3.rotate(self.target_rpm[2], self.current_rpm[2])

    def cleanup(self):
        """Properly shuts down motors, GPIO, and serial communication"""
        self.get_logger().info("Shutting down. Cleaning up resources...")
        self.motor1.destroy()
        self.motor2.destroy()
        self.motor3.destroy()
        self.ser.close()  # Close serial communication
        self.get_logger().info("Serial port closed")
        GPIO.cleanup()  # Cleanup GPIO only once
        self.get_logger().info("GPIO cleaned up")

def main(args=None):
    rclpy.init(args=args)
    node = BotDirectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()