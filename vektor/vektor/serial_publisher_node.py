import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import pi, sin, cos

import serial
import struct

# Open the serial port (adjust the port and baud rate as needed)
ser = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=1)

def read_floats():
    # Read 12 bytes (3 floats x 4 bytes each)
    data = ser.read(12)
    # Check if enough data was received
    if len(data) == 12:
        # Unpack the 12 bytes into three floats using the same format '<fff'
        return struct.unpack('<fff', data)
    else:
        return None


class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        print("serial_publisher_node publishing to topic /serial_data")
        # Create a publisher for the float array data on the 'serial_data' topic
        self.publisher_ = self.create_publisher(Float32MultiArray, 'serial_data', 10)

        # Open the serial port.
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.1)
            self.get_logger().info("Serial port opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # Create a timer that periodically checks the serial port (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # We expect 12 bytes (3 floats, 4 bytes each)
        expected_bytes = 12

        # Check if enough data is available in the serial buffer
        if self.serial_port.in_waiting >= expected_bytes:
            data = self.serial_port.read(expected_bytes)
            if len(data) == expected_bytes:
                try:
                    # Unpack 3 floats from the binary data.
                    # '<fff' means little-endian and 3 floats.
                    float_values = struct.unpack('<fff', data)
                except struct.error as e:
                    self.get_logger().error(f"Error unpacking data: {e}")
                    return

                # Create and populate a Float32MultiArray message
                msg = Float32MultiArray()
                msg.data = list(float_values)

                # Publish the message
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {msg.data}")
            else:
                self.get_logger().warning("Incomplete data packet received.")
        else:
            self.get_logger().debug("Not enough data available in serial buffer.")

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()