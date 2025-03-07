# import rclpy
# from rclpy.node import Node
# import serial
# from wheel_sensors_interfaces.msg import WheelDistances

# class SerialPulseReaderNode(Node):
#     def __init__(self):
#         super().__init__('distance_reader')
#         self.usb_serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0)
#         self.publisher = self.create_publisher(WheelDistances, '/bot/wheel_distances', 10)
#         self.timer = self.create_timer(0.2, self.publish_distance)
#         # self.last_pulse_left = self.last_pulse_right = self.last_pulse_back = 0
#         self.last_pulses = [0, 0, 0]
#         self.distance_per_pulse = 12.35/1000 #meter
#         self.get_logger().info("node started")
        
#     def publish_distance(self):
#         # try:
#         pulse_data = self.usb_serial.readline().decode().strip()
#         if pulse_data:
#             try:
#                 accumulated_pulses = pulse_data.split(',')
#                 accumulated_pulses = [int(x) for x in accumulated_pulses]
#                 instant_pulse_l = accumulated_pulses[0] - self.last_pulses[0]
#                 instant_pulse_r = accumulated_pulses[1] - self.last_pulses[1]
#                 instant_pulse_b = accumulated_pulses[2] - self.last_pulses[2]
#                 self.last_pulses = accumulated_pulses
                
#                 msg = WheelDistances()
#                 msg.dead_wheel_left = float(accumulated_pulses[0]) * instant_pulse_l * self.distance_per_pulse
#                 msg.dead_wheel_right = float(accumulated_pulses[1]) * instant_pulse_r * self.distance_per_pulse
#                 msg.dead_wheel_back = float(accumulated_pulses[2]) * instant_pulse_b * self.distance_per_pulse

#                 self.publisher.publish(msg)
#                 # self.get_logger().info(f'Published Distances: {msg.dead_wheel_left}, {msg.dead_wheel_right}, {msg.dead_wheel_back}')
#                 self.get_logger().info(f"{accumulated_pulses}")
#             except Exception as e:
#                 self.get_logger().warning(f'Error reading Distance data {e}:')
#         # else:


# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialPulseReaderNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
import serial
from wheel_sensors_interfaces.msg import WheelDistances, WheelPulses

class SerialPulseReaderNode(Node):
    def __init__(self):
        super().__init__('distance_reader')

        try:
            self.usb_serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0)
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            return

        self.publisher = self.create_publisher(WheelPulses, '/bot/tracking_wheel_pulses', 10)
        self.create_timer(0.1, self.publish_pulses)

        self.last_pulses = [0, 0, 0]
        self.DIST_PER_PULSE = 12.35 / 1000  # Meters per pulse

        self.old_log_msg = ''

    def publish_pulses(self):
        """Reads pulse data from serial and publishes wheel distances."""
        try:
            pulse_data = self.usb_serial.readline().decode().strip()
            if not pulse_data:
                return

            try:
                accumulated_pulses = list(map(int, pulse_data.split(',')))
                if len(accumulated_pulses) != 3:
                    return  # Ignore malformed data
                
                msg = WheelPulses() # TODO: change to WheelPulses
                msg.pulse_left, msg.pulse_right, msg.pulse_back = accumulated_pulses[0], accumulated_pulses[1], accumulated_pulses[2]
                self.publisher.publish(msg)
                self.last_pulses = accumulated_pulses  # Update last pulses
                
                new_log_msg=f"{accumulated_pulses}"
                if self.old_log_msg != new_log_msg:
                    self.get_logger().info(new_log_msg)
                    self.old_log_msg = new_log_msg
                # self.get_logger().info(new_log_msg)
            except ValueError:
                return  # Ignore non-integer data

            # Compute distances in one step
            # msg = WheelDistances()
            # msg.dead_wheel_left, msg.dead_wheel_right, msg.dead_wheel_back = [
            #     new * (new - old) * self.DIST_PER_PULSE 
            #     for new, old in zip(accumulated_pulses, self.last_pulses)
            # ]
           


        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def destroy_node(self):
        """Ensures serial port is closed on shutdown."""
        if hasattr(self, 'usb_serial') and self.usb_serial.is_open:
            # self.usb_serial.close()
            # self.get_logger().info("Serial connection closed.")
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialPulseReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# import serial
# from wheel_sensors_interfaces.msg import WheelPulses

# class SerialPulseReaderNode(Node):
#     def __init__(self):
#         super().__init__('distance_reader')

#         try:
#             self.usb_serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)
#             self.get_logger().info("Serial connection established.")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to connect to serial port: {e}")
#             return

#         self.publisher = self.create_publisher(WheelPulses, '/bot/tracking_wheel_pulses', 10)

#         self.last_pulses = [0, 0, 0]
#         self.old_log_msg = ''

#         self.read_serial_data()  # Start listening for serial data

#     def read_serial_data(self):
#         """Continuously listens for valid serial data and publishes only when valid data is received."""
#         while rclpy.ok():
#             try:
#                 pulse_data = self.usb_serial.readline().decode().strip()
#                 if not pulse_data:
#                     continue  # Skip if no data

#                 try:
#                     accumulated_pulses = list(map(int, pulse_data.split(',')))
#                     if len(accumulated_pulses) != 3:
#                         continue  # Ignore malformed data
#                 except ValueError:
#                     continue  # Ignore non-integer data

#                 # ✅ Ignore repeated values
#                 if accumulated_pulses == self.last_pulses:
#                     continue  

#                 # ✅ Publish only when valid new data is received
#                 msg = WheelPulses()
#                 msg.pulse_left, msg.pulse_right, msg.pulse_back = accumulated_pulses
#                 self.publisher.publish(msg)

#                 # # ✅ Log only if new data is different from the previous log
#                 new_log_msg = f"{accumulated_pulses}"
#                 # if self.old_log_msg != new_log_msg:
#                 #     self.get_logger().info(new_log_msg)
#                 #     self.old_log_msg = new_log_msg
#                 self.get_logger().info(new_log_msg)


#                 # ✅ Update last pulses
#                 self.last_pulses = accumulated_pulses  

#             except serial.SerialException as e:
#                 self.get_logger().error(f"Serial communication error: {e}")

#     def destroy_node(self):
#         """Ensures serial port is closed on shutdown."""
#         if hasattr(self, 'usb_serial') and self.usb_serial.is_open:
#             self.usb_serial.close()
#             self.get_logger().info("Serial connection closed.")
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = SerialPulseReaderNode()

#     try:
#         rclpy.spin(node)  # Keep the ROS node running
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
