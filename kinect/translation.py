import math

def get_angles(x, y, z):
    x_of = 0.284
    y_of = 0.365
    z_of = 0.167
    z_blaster = z -z_of
    y_blaster = y - y_of
    x_blaster = x + x_of

    theta=math.atan(z_blaster/x_blaster)*360/2*math.pi
    phi=math.atan(y_blaster/x_blaster)
    return [theta,phi]