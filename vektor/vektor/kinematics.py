from math import sin, cos, pi, radians, degrees

wheel_radius = 0.029 # meters
bot_radius = 0.075 # meters
bot_velocity = wheel_radius * (180 * 2 * pi / 60)
#bot_velocity = 1
rpm_factor = 1

def target_wheel_rpm(theta, wz) -> tuple[float, float, float]:

    theta = radians(theta)
    print(bot_velocity)

    #wz = -pi if wbz == -1 else pi # TODO check this

    vy = sin(theta) * bot_velocity
    vx = cos(theta) * bot_velocity

    u1 = round((-bot_radius*wz + vx) / wheel_radius * rpm_factor, 4)# how fast wheel 1 must rotate
    u2 = round((-bot_radius*wz - cos(pi/3)*vx - sin(pi/3)*vy) / wheel_radius * rpm_factor, 4)
    u3 = round((-bot_radius*wz - cos(pi/3)*vx + sin(pi/3)*vy) / wheel_radius * rpm_factor, 4)
    
    return u1, u2, u3 # return the target rpm of each wheel 

print(target_wheel_rpm(90, 0))