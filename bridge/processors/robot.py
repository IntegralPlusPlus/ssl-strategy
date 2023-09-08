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

    def go_to_point(self, point, is_final_point):
        pass

        if const.IS_SIMULATOR_USED:
            self.speedY *= -1

    def get_speed(self, distance):
        pass


    def rotate_to_point(self, point):
        pass
        if const.IS_SIMULATOR_USED:
            pass