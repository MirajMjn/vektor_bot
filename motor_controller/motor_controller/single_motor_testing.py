import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from motor_controller.utils import target_wheel_rpm
from motor_controller.motor2 import Motor

ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=0)


class PwmSubscriberNode(Node):
    def __init__(self):
        super().__init__('bot_direction_subscriber') # name of node
        print("subscriber initialized, !!!!!!!RUN CODE ON THONNY!!!!!!!!!!")

        self.subscription = self.create_subscription(msg_type=String, topic='/bot_direction', callback=self.listener_callback, qos_profile=10) # subscribe topic /led_control using String type, queue size is 10
        self.rpm_subscription = self.create_subscription(String, '/rpm', self.rpm_callback, 10)
        self.motor1 = Motor(5, 6, name='motor1') # physical 29, 31 
        # self.motor2 = Motor(27, 17, 'motor2')  # physical 11, 13
        # self.motor3 = Motor(23, 24, 'motor3') # physical 16 18

    def stop_motors(self):
        self.motor1.stop()
        # self.motor2.stop()
        # self.motor3.stop()
    def rpm_callback(self):
        pass

    def rpm_subscription(self, rpms):
        rpms = rpms.data.split(',')  # "230,200,100"
        rpm1 = float(rpms[0])


    def listener_callback(self, msg):
        theta = float(msg.data.split(',')[0]) 
        wbz = float(msg.data.split(',')[1])
        # print(f"theta:{theta}, omega:{wbz}")
        start = True
        if theta == 501 and wbz == 501:
            start = False
            self.stop_motors()
            return

        if start:
            
            rpms_pico = ser.readline().decode().strip()  # Read and decode
            u1, u2, u3 = target_wheel_rpm(theta, wbz)
            print(f"u1:{u1:.2f}\t u2:{u2:.2f}\t u3:{u3:.2f}")


            if rpms_pico:
                
                print("angle", theta, wbz)
                    # rpm1, rpm2, rpm3 = [float(rpm) for rpm in rpms_pico.split(',')] #is in pwm value 
                print(rpms_pico)
                rpm1 = [float(rpm) for rpm in rpms_pico.split(',')][0] #is in pwm value 

                print(f"r1:{rpm1}")
                print()
                
            # except ValueError:
            #     print("\n\nmissed an rpm measurement\n\n")
            #     return

                self.motor1.rotate(u1, rpm1)

                # self.motor2.rotate(u2, rpm2)
                # self.motor3.rotate(u3, rpm3)


    def __del__(self):  # destructor for motor class, How does it work?
        self.motor1.destroy()
        # self.motor2.destroy()
        # self.motor3.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = PwmSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()