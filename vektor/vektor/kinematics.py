from math import sin, cos, pi, radians, degrees

wheel_radius = 59/ (2* 1000) # meters
bot_radius = 138 /1000 # meters
# bot_velocity = wheel_radius * (180 * 2 * pi / 60)

rotate_rpm = 200
pose = 0
sl, sb, sr = (0.067, 0.079, 0.067) # distance of three wheels from U shape center
offset = 10.93 # mm to be subtracted
x_old = 0 
y_old = 0

rotation_speed = 1.7

rpm_factor = 60 / (2 * pi)
bot_velocity = wheel_radius * (180 * 2 * pi / 60)

def target_wheel_rpm(theta, wz, magnitude) -> tuple[float, float, float]:
    # if wz == 1:
    #     wz = rotation_speed
    # if wz == -1:
    #     wz = -rotation_speed
    wz = wz * rotation_speed # because wz is +1 or -1
    theta = radians(theta)
    # wz = wz/bot_radius
    bot_velocity = wheel_radius * (180 * 2 * pi / 60)

    if magnitude == 0:
        bot_velocity =0

    vy = sin(theta) * bot_velocity
    vx = cos(theta) * bot_velocity

    u1 = round((-bot_radius*wz + vx) / wheel_radius * rpm_factor, 4)# how fast wheel 1 must rotate
    u2 = round((-bot_radius*wz - cos(pi/3)*vx - sin(pi/3)*vy) / wheel_radius * rpm_factor, 4)
    u3 = round((-bot_radius*wz - cos(pi/3)*vx + sin(pi/3)*vy) / wheel_radius * rpm_factor, 4)
    
    return u1, u2, u3 # return the target rpm of each wheel 


def odometry(del_l, del_r, del_b):
    global pose ,x_old, y_old
    del_pose = (del_r - del_l) / (sr + sl)
    rel_del_x = (del_r + del_l) / 2
    rel_del_y = del_b - sb * del_pose

    pose += del_pose

    x = x_old + cos(pose) * rel_del_x - sin(pose) * rel_del_y
    y = y_old + sin(pose) * rel_del_x + cos(pose) * rel_del_y

    x_old = x
    y_old = y

    return x*100, y*100, degrees(pose) # in meter, degrees

