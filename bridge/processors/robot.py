import math
import bridge.processors.auxiliary as auxiliary
import bridge.processors.const as const

class Robot:
    def __init__(self, color, r_id, x, y, orientation):
        self.rId = r_id
        self.isUsed = 1
        self.x = x
        self.y = y
        self.orientation = orientation
        self.maxSpeed = const.MAX_SPEED
        self.maxSpeedR = const.MAX_SPEED_R
        self.color = color

        # Changeable params
        self.speedX = 0.0
        self.speedY = 0.0
        self.speedR = 0.0
        self.kickUp = 0.0
        self.kickForward = 0.0
        self.autoKick = 0.0
        self.kickerVoltage = const.BASE_KICKER_VOLTAGE
        self.dribblerEnable = 0.0
        self.speedDribbler = 0.0
        self.kickerChargeEnable = 0.0
        self.beep = 0.0
        self.acc = const.ACCELERATION

    def update(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.kickForward = 0
        self.kickUp = 0

    def kick_forward(self):
        self.kickForward = 1

    def kick_up(self):
        self.kickUp = 1

    def go_to_point_with_detour(self, target_point, y_team, b_team):
        pass

    def go_to_point(self, point):
        g = math.sqrt((point.x - self.x)**2 +  (point.y - self.y)**2)
        if g == 0: g = 1
        b = math.acos((point.x - self.x)/g)
        cosRazn = ((point.x - self.x) * math.cos(self.orientation) + (point.y - self.y) * math.sin(self.orientation)) / g
        sinRazn = ((point.y - self.y) * math.cos(self.orientation) - (point.x - self.x) * math.sin(self.orientation)) / g
        self.speedX = cosRazn * 0.025 * g #0.03
        self.speedY = sinRazn * 0.025 * g #0.03
        #self.orientation  

        if const.IS_SIMULATOR_USED:
            self.speedY *= -1

    def get_speed(self, distance):
        pass


    def rotate_to_point(self, point):
        (vecX1, vecY1) = (point.x - self.x, point.y - self.y)
        (vecX2, vecY2) = (1, 0)#(math.cos(self.orientation), math.sin(self.orientation))
        d1 = math.sqrt(vecX1**2 + vecY1**2)
        d2 = math.sqrt(vecX2**2 + vecY2**2)
        angle = math.acos((vecX1*vecX2 + vecY1*vecY2) / (d1 * d2))
        if vecY1 < 0: angle *= -1
        
        razn = abs(angle - self.orientation)
        if razn < 2*math.pi - razn: self.speedR = (angle - self.orientation) * 20#16
        else: self.speedR = (angle - self.orientation - 2*math.pi) * 20

        if const.IS_SIMULATOR_USED:
            pass