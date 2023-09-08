import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def vect_mult(v, u):
    return v.x * u.x + v.y * u.y


def scal_mult(v, u):
    return v.x * u.y - v.y * u.x


def format_angle(ang):
    while ang > math.pi:
        ang -= 2 * math.pi
    while ang < -math.pi:
        ang += 2 * math.pi
    return ang