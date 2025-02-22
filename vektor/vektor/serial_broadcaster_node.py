import rclpy
from rclpy.node import Node
from my_interfaces.msg import VektorRPM

import serial
import struct

class SerialBoradcasterNode(Node):
    def __init__(self):
        super().__init__('serial_broadcaster_node')
        self.publisher_ = self.create_publisher(VektorRPM, 'serial_data', 2)
        self.get_logger().info(f"serial_boradcaster_node publishing in topic: /serial_data")

        # Open the serial port.
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.1) #TODO : verify seial port
            self.get_logger().info("Serial port opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port [Exception] : {e}")
            raise e
        
        # Create a timer that periodically checks the serial port (every 0.1 seconds)
        self.timer = self.create_timer(0.4, self.timer_callback)

    def timer_callback(self):
        # We expect 24 bytes (6 floats, 4 bytes each)
        expected_bytes = 24

        # Check if enough data is available in the serial buffer
        if self.serial_port.in_waiting >= expected_bytes:
            data = self.serial_port.read(expected_bytes)
            if len(data) == expected_bytes:
                try:
                    # Unpack 6 floats from the binary data.
                    # '<ffffff' means little-endian and 6 floats.
                    float_values = struct.unpack('<ffffff', data)
                except struct.error as e:
                    self.get_logger().error(f"Error unpacking data: {e}")
                    return

                # Create and populate message
                msg = VektorRPM()
                msg.rpm = list(float_values) #msg.rpm is a float32[]

                # Publish the message
                self.publisher_.publish(msg)
            else:
                self.get_logger().warning("Incomplete data packet received.")
                pass
        else:
            self.get_logger().warning("Not enough data available in serial buffer.")
            pass

    def destroy_node(self):
        # Close the serial port when shutting down the node
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBoradcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()