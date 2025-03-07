# import rclpy
# from rclpy.node import Node
# from wheel_sensors_interfaces.msg import WheelDistances, OdomData, WheelPulses
# import math
# from math import cos, sin, atan2, degrees, pi

# # DIST_PER_PULSE = 12.35 #mm actual theoretical
# ERROR_FACTOR = 1.11 # pulses per pulse
# WHEEL_DIAMETER = 58 #mm
# # DPP_PRACTICAL_L = pi * WHEEL_DIAMETER / 11.40 #mm practical observed from data collection and averaging
# # DPP_PRACTICAL_R =  pi * WHEEL_DIAMETER / 11.54
# # DPP_PRACTICAL_B =  pi * WHEEL_DIAMETER / 11.46

# DPP_PRACTICAL_L = pi * WHEEL_DIAMETER / 11 #mm practical observed from data collection and averaging
# DPP_PRACTICAL_R =  pi * WHEEL_DIAMETER / 11
# DPP_PRACTICAL_B =  pi * WHEEL_DIAMETER / 11

# heading = 0

# def normalize_angle(angle):
#     return math.atan2(math.sin(angle), math.cos(angle))


# class OdometryPublisherNode(Node):
#     def __init__(self):
#         super().__init__('odometry_publisher')
#         self.publisher = self.create_publisher(OdomData, '/bot/odometry', 10)
#         self.subscriber = self.create_subscription(WheelPulses, '/bot/tracking_wheel_pulses', self.tracking_wheels_callback, 1)
#         self.get_logger().info('Odometry publisher started....')
#         self.x = self.y = self.pose = 0
#         self.x_pos = 0
#         self.y_pos = 0
#         self.SL, self.SB, self.SR = (67, 79, 67) # MM distance of three wheels from U shape center
#         self.pulse_record = [0, 0, 0]
#         self.OFFSET = 10.93 # mm
#         self.forward_offset = 10.93 #mm
#         self.trackwidth = self.SL + self.SR
#         self.heading = 0

#         self.old_log_msg = ''
#         # self.runcount = 0
#         # self.stale_b, self.stale_l, self.stale_r = 0, 0, 0

#     # def first_run_reset(self, msg):
#     #     if self.runcount == 0:
#     #         self.stale_l, self.stale_r, self.stale_b = msg.pulse_left, msg.pulse_right, msg.pulse_back
#     #         self.pulse_record = [0, 0, 0]
#     #         self.runcount = 1
#     #     else:
#     #         self.stale_l, self.stale_r, self.stale_b = 0, 0, 0

#     def tracking_wheels_callback(self, msg):
#         # self.first_run_reset(msg)

#         #To ignore updates if pulses haven't changed (robot is not moving)
#         if (msg.pulse_left == self.pulse_record[0] and
#             msg.pulse_right == self.pulse_record[1] and
#             msg.pulse_back == self.pulse_record[2]):
#             return  # Stop processing if no new movement

#         # del_l = (msg.pulse_left - self.pulse_record[0]) * DPP_PRACTICAL_L # v1
#         # del_r = (msg.pulse_right - self.pulse_record[1]) * DPP_PRACTICAL_R #v2
#         # del_b = (msg.pulse_back - self.pulse_record[2]) * DPP_PRACTICAL_B #v3
#         # self.pulse_record[0], self.pulse_record[1], self.pulse_record[2] = msg.pulse_left, msg.pulse_right, msg.pulse_back

#         # del_pose = (del_l - del_) / (self.SR + self.SL) #tick omega
#         # print(f"delpose={del_pose}, pose={self.pose}")
#         # rel_del_x = (del_r + del_l) / 2 # tick vx
#         # rel_del_y = del_b - self.SB * del_pose # tick 

#         # self.pose = self.pose + del_pose

#         # self.x += cos(self.pose) * rel_del_x - sin(self.pose) * rel_del_y
#         # self.y += sin(self.pose) * rel_del_x + cos(self.pose) * rel_del_y

#         left_encoder_pos = msg.pulse_left
#         right_encoder_pos = msg.pulse_right
#         center_encoder_pos = msg.pulse_back
#         delta_left_encoder_pos = left_encoder_pos - self.pulse_record[0]
#         delta_right_encoder_pos = right_encoder_pos - self.pulse_record[1]
#         delta_center_encoder_pos = center_encoder_pos - self.pulse_record[2]

#         # delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos
#         # delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos
#         # delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos

#         phi = (delta_left_encoder_pos - delta_right_encoder_pos) / self.trackwidth
        
#         delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2
#         delta_perp_pos = delta_center_encoder_pos - self.forward_offset * phi

#         delta_x = delta_middle_pos * cos(heading) - delta_perp_pos * sin(heading)
#         delta_y = delta_middle_pos * sin(heading) + delta_perp_pos * cos(heading)

#         self.x_pos += delta_x
#         self.y_pos += delta_y
#         self.heading += phi

#         self.pulse_record = [left_encoder_pos, right_encoder_pos, center_encoder_pos]

#         odom_msg = OdomData()
#         odom_msg.x = x_pos
#         odom_msg.y = y_pos
#         odom_msg.pose = heading

#         # odom_msg = OdomData()
#         # odom_msg.x = self.x
#         # odom_msg.y = self.y
#         # odom_msg.pose = self.pose

#         self.publisher.publish(odom_msg)
#         new_log_msg = f'Odometry: {odom_msg.x/10:.1f} cm, {odom_msg.y/10:.2f} cm, {degrees(odom_msg.pose):.2f} deg'
#         if self.old_log_msg != new_log_msg:
#             self.get_logger().info(new_log_msg)
#             self.old_log_msg = new_log_msg


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdometryPublisherNode()
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
from wheel_sensors_interfaces.msg import WheelDistances, OdomData, WheelPulses
import math
from math import cos, sin, atan2, degrees, pi

# DIST_PER_PULSE = 12.35 #mm actual theoretical
ERROR_FACTOR = 1.11 # pulses per pulse ie 11% pulses are lost 
WHEEL_DIAMETER = 58 #mm
# DPP_PRACTICAL_L = pi * WHEEL_DIAMETER / 11.40 #mm practical observed from data collection and averaging
# DPP_PRACTICAL_R =  pi * WHEEL_DIAMETER / 11.54
# DPP_PRACTICAL_B =  pi * WHEEL_DIAMETER / 11.46

DPP_PRACTICAL_L = pi * WHEEL_DIAMETER / 11 #mm practical observed from data collection and averaging
DPP_PRACTICAL_R =  pi * WHEEL_DIAMETER / 11
DPP_PRACTICAL_B =  pi * WHEEL_DIAMETER / 11


class OdometryPublisherNode(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher = self.create_publisher(OdomData, '/bot/odometry', 10)
        self.subscriber = self.create_subscription(WheelPulses, '/bot/tracking_wheel_pulses', self.tracking_wheels_callback, 1)
        self.get_logger().info('Odometry publisher started....')

        # Position and Heading
        self.x_pos = 0
        self.y_pos = 0
        self.heading = 0

        # Robot Dimensions
        self.SL, self.SB, self.SR = (67, 79, 67) # MM distance of three wheels from center
        self.trackwidth = self.SL + self.SR
        # self.forward_offset = 10.93 / DPP_PRACTICAL_B  # Convert to pulses
        self.forward_offset = self.SB   # Convert to pulses

        # Encoder history
        self.pulse_record = [0, 0, 0]
        self.old_log_msg = ''

    def tracking_wheels_callback(self, msg):
        # Ignore if no pulses changed
        if (msg.pulse_left == self.pulse_record[0] and
            msg.pulse_right == self.pulse_record[1] and
            msg.pulse_back == self.pulse_record[2]):
            return

        # Convert pulses to mm
        delta_left_encoder_pos = ((msg.pulse_left - self.pulse_record[0]) * DPP_PRACTICAL_L) * ERROR_FACTOR # is distance
        delta_right_encoder_pos = (msg.pulse_right - self.pulse_record[1]) * DPP_PRACTICAL_R * ERROR_FACTOR # 
        delta_center_encoder_pos = (msg.pulse_back - self.pulse_record[2]) * DPP_PRACTICAL_B * ERROR_FACTOR # is distance

        phi = (delta_left_encoder_pos - delta_right_encoder_pos) / self.trackwidth
        delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2
        delta_perp_pos = delta_center_encoder_pos - self.forward_offset * phi

        delta_x = delta_middle_pos * math.cos(self.heading) - delta_perp_pos * math.sin(self.heading)
        delta_y = delta_middle_pos * math.sin(self.heading) + delta_perp_pos * math.cos(self.heading)

        # Update position and heading
        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading += phi

        # Save new pulse values
        self.pulse_record = [msg.pulse_left, msg.pulse_right, msg.pulse_back]

        # Publish updated odometry
        odom_msg = OdomData()
        odom_msg.x = self.x_pos
        odom_msg.y = self.y_pos
        odom_msg.pose = self.heading

        self.publisher.publish(odom_msg)

        # Log update
        new_log_msg = f'Odometry: {odom_msg.x/10:.1f} cm, {odom_msg.y/10:.2f} cm, {math.degrees(odom_msg.pose):.2f} deg'
        if self.old_log_msg != new_log_msg:
            self.get_logger().info(new_log_msg)
            self.old_log_msg = new_log_msg


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
