
import rclpy
from rclpy.node import Node
import serial
from wheel_sensors_interfaces.msg import OdomData

class SerialOdometryReaderNode(Node):
    def __init__(self):
        super().__init__('Odometry_Reader')

        try:
            self.usb_serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0)
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            return

        self.publisher = self.create_publisher(OdomData, '/bot/Odometry', 10)
        self.create_timer(0.2, self.publish_odometry)
        self.old_log_msg = ''

    def publish_odometry(self):
        """Reads odometry data from serial and publishes."""
        try:
            xyh_data = self.usb_serial.readline().decode().strip()
            if not xyh_data:
                # print('no xyh data from serial usb port')
                return

            try:
                xyh = list(map(float, xyh_data.split(',')))
                if len(xyh) != 6:
                    return  # Ignore malformed data
                
                msg = OdomData() 
                msg.x, msg.y, msg.pose, c = xyh[0], xyh[1], xyh[2]
                self.publisher.publish(msg)                
                new_log_msg=f"{xyh}"
                # if self.old_log_msg != new_log_msg:
                self.get_logger().info(new_log_msg)
                    # self.old_log_msg = new_log_msg
                # self.get_logger().info(new_log_msg)
            except ValueError:
                # print('value error')
                return  # Ignore non-float data

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
    node = SerialOdometryReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
