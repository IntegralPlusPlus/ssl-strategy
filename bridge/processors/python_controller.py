import attr
import numpy as np
import typing
import struct
import time
import bridge.processors.const as const
import bridge.processors.robot as robot
import bridge.processors.team as team
import queue

from bridge.bus import DataReader, DataWriter
from bridge.common import config
#from bridge.matlab.engine import matlab_engine
from bridge.model.referee import RefereeCommand
from bridge.processors import BaseProcessor
import math
import bridge.processors.auxiliary as auxiliary

# TODO: Refactor this class and corresponding matlab scripts
@attr.s(auto_attribs=True)
class MatlabController(BaseProcessor):
    xR = 4500
    yR = 0

    processing_pause: typing.Optional[float] = 0.001
    max_commands_to_persist: int = 20
    state = 0

    wasBallKicked = False
    firstBallX = -6000

    vision_reader: DataReader = attr.ib(init=False, default=DataReader(config.VISION_DETECTIONS_TOPIC))
    referee_reader: DataReader = attr.ib(init=False, default=DataReader(config.REFEREE_COMMANDS_TOPIC))
    commands_writer: DataWriter = attr.ib(init=False)

    cur_time = time.time()
    dt = 0
    ball = auxiliary.Point(0, 0)
    count = 0

    b_team = team.Team(const.GK)
    y_team = team.Team(const.ENEMY_GK)
    for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        y_team.add_robot(robot.Robot('y', i, 10e10, 10e10, 0))
    for i in range(const.TEAM_ROBOTS_MAX_COUNT):
        b_team.add_robot(robot.Robot('b', i, 10e10, 10e10, 0))

    def __attrs_post_init__(self):
        self.commands_writer = DataWriter(config.ROBOT_COMMANDS_TOPIC, self.max_commands_to_persist)

    def get_last_referee_command(self) -> RefereeCommand:
        referee_commands = self.referee_reader.read_new()
        if referee_commands:
            return referee_commands[-1]
        return RefereeCommand(0, 0, False)

    def returnFirstElement(a):
        return a[0]
    
    def intersection(x, y, xFrom, yFrom, xTo, yTo):
        a = 1 / (xTo - xFrom)
        b = -1 / (yTo - yFrom)
        c = yFrom / (yTo - yFrom) - xFrom / (xTo - xFrom)
        distPointLine = math.abs(a * x + b * y + c) / math.sqrt(a**2 + b**2) 
        
        return distPointLine < 220

    async def process(self) -> None:
        print("GO")
        balls = np.zeros(const.BALL_PACKET_SIZE * const.MAX_BALLS_IN_FIELD)
        field_info = np.zeros(const.GEOMETRY_PACKET_SIZE)
        ssl_package = 0
        try:
            ssl_package = self.vision_reader.read_new()[-1]
        except: None

        if ssl_package:   
            #print("SECOND")         
            geometry = ssl_package.geometry
            if geometry:
                field_info[0] = geometry.field.field_length
                field_info[1] = geometry.field.field_width

            detection = ssl_package.detection
            camera_id = detection.camera_id
            for ball_ind, ball in enumerate(detection.balls):
                balls[ball_ind + (camera_id - 1) * const.MAX_BALLS_IN_CAMERA] = camera_id
                balls[ball_ind + const.MAX_BALLS_IN_FIELD + (camera_id - 1) * const.MAX_BALLS_IN_CAMERA] = ball.x
                balls[ball_ind + 2 * const.MAX_BALLS_IN_FIELD + (camera_id - 1) * const.MAX_BALLS_IN_CAMERA] = ball.y
                self.ball = auxiliary.Point(ball.x, ball.y)

            # TODO: Barrier states
            for robot in detection.robots_blue:
                self.b_team.robot(robot.robot_id).update(robot.x, robot.y, robot.orientation)

            for robot in detection.robots_yellow:
                self.y_team.robot(robot.robot_id).update(robot.x, robot.y, robot.orientation)

            # referee_command = self.get_last_referee_command()
            rules = []

            #self.b_team.robot(0).rotate_to_point(auxiliary.Point(self.xR, self.yR))
            # Strategy code start
            if self.state == 0: self.b_team.robot(0).rotate_to_point(auxiliary.Point(self.ball.x, self.ball.y))
            elif self.state == 1: self.b_team.robot(0).rotate_to_point(auxiliary.Point(self.xR, self.yR))
            R = 200
            r = 110
            #xR = 4500
            #yR = 0
            x1 = self.b_team.robot(0).x
            y1 = self.b_team.robot(0).y
            x0 = self.ball.x
            y0 = self.ball.y
            goalUp = 500
            goalDown = -500
            n = 6

            xAnti = []
            yAnti = []
            xRobots = []
            yRobots = []
            xAnti.append(4500)
            yAnti.append(goalDown)

            for i in range(n):
                xAnti.append(self.y_team.robot(i).x)
                yAnti.append(self.y_team.robot(i).y)
                xRobots.append(self.y_team.robot(i).x)
                yRobots.append(self.y_team.robot(i).y)
                #xAnti = (self.y_team.robot(0).x, self.y_team.robot(1).x)
                #yAnti = (self.y_team.robot(0).y, self.y_team.robot(1).y)
            
            for i in range(1, n):
                xRobots.append(self.b_team.robot(i).x)
                yRobots.append(self.b_team.robot(i).y)

            xAnti.append(4500)
            yAnti.append(goalUp)
            
            #up = []
            #down = []
            central = []
            #print("SECOND")
            for i in range(n + 2):
                dist = math.sqrt((x0 - xAnti[i])**2 + (y0 - yAnti[i])**2)
                D = dist * (4500 - x0) / (xAnti[i] - x0)
                if (R + r + 20) / dist > 1: alphaNew = math.asin(1)
                else: alphaNew = math.asin((R + r + 20) / dist)
                #print(4500 - x1, D, (xAnti[0] - x1) / dist)
                gamma = math.acos((xAnti[i] - x0) / dist) - alphaNew
                
                downDist = math.sqrt(D**2 - (4500 - x0)**2) - (4500 - x0) * math.tan(gamma) #HASHUV
                
                if abs(D * math.sin(alphaNew) / downDist) <= 1: 
                    bettaNew = math.asin(D * math.sin(alphaNew) / downDist) 
                elif math.sin(alphaNew) > 0: bettaNew = math.asin(1)
                else: bettaNew = math.asin(-1)
                
                upDist = D * math.sin(alphaNew) / math.sin(math.pi - 2 * alphaNew - bettaNew) #HASHUV

                if y0 > yAnti[i]: 
                    (downDist, upDist) = (upDist, downDist)
                    ycc = y0 - D * math.sin(alphaNew + gamma)
                else:
                    ycc = y0 + D * math.sin(alphaNew + gamma)


                #up.append(ycc + upDist)
                #down.append(ycc - downDist)
                if ycc < goalDown or ycc > goalUp:
                    central.append([ycc, ycc + upDist, ycc - downDist])
            #if yAnti[0] > yAnti[1]:
            #    pUp = goalUp - up[0]
            #    pMiddle = down[0] - up[1]
            #    pDown = down[1] - goalDown
            #else:
            #    pUp = goalUp - up[1]
            #    pMiddle = down[1] - up[0]
            #    pDown = down[0] - goalDown
            #maxi = max(pUp, pMiddle, pDown)
            
            #print("SECOND")
            central = sorted(central, key=lambda x: x[0])
            #for i in range(n + 2):
            #    print(central[i][0], end = " ")
            #    print(central[i][0], central[i][1], central[i][2], end = " ")
            #print()

            maxiAngle = 0
            rememberI = -1
            for i in range(len(central) - 1):
                lookUp = central[i + 1][2]
                lookDown = central[i][1]

                bokDown = math.sqrt((x1 - 4500)**2 + (y1 - lookDown)**2)
                bokUp = math.sqrt((x1 - 4500)**2 + (y1 - lookUp)**2)
                v1 = (4500 - x1, lookDown - y1)
                v2 = (4500 - x1, lookUp - y1)
                #if (v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp) > 1: 
                #    angleBetweenVectors = math.acos(1)
                #elif (v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp) < -1:
                #     angleBetweenVectors = math.acos(-1)
                angleBetweenVectors = math.acos((v1[0] * v2[0] + v1[1] * v2[1]) / (bokDown * bokUp))

                if angleBetweenVectors > maxiAngle:
                    maxiAngle = angleBetweenVectors
                    rememberI = i
                #osn = lookUp - lookDown
                #distUp = osn * bokUp / (bokUp + bokDown)

            #print("SECOND")
            if rememberI != -1:
                lookUp = central[rememberI + 1][2]
                lookDown = central[rememberI][1]
                bokDown = math.sqrt((x1 - 4500)**2 + (y1 - lookDown)**2)
                bokUp = math.sqrt((x1 - 4500)**2 + (y1 - lookUp)**2)
                
                osn = lookUp - lookDown
                distUp = osn * bokUp / (bokUp + bokDown)
                
                self.xR = 4500
                self.yR = lookUp - distUp
            else:
                self.xR = 4500
                self.yR = 0
            #print()
            #print("UP: ", up, "DOWN: ", down, "XR: ", self.xR, "YR: ", self.yR);
            #self.state = 1

            if self.ball.x - self.xR != 0: k = (self.ball.y - self.yR) / (self.ball.x - self.xR)
            else: k = 0
            b = self.yR - k * self.xR

            xTo = self.ball.x - (R + r + 20)
            yTo = k * xTo + b

            if x1 > x0 and abs(x1 - x0) > (R + r + 20): #D > 50:
                d = math.sqrt((x0 - x1)**2 + (y0 - y1)**2)
                rr = R + r + 300 #2
                if rr / d > 1: d = rr
                if d**2 - rr**2 >= 0: hyp = math.sqrt(d**2 - rr**2)
                else: hyp = 0

                alpha = math.asin(rr / d)
                sigma2 = math.atan2(abs(y0 - y1), abs(x0 - x1)) - alpha
                if y0 > y1: yTo = y1 + hyp * math.sin(sigma2)
                else: yTo = y1 - hyp * math.sin(sigma2)
                xTo = x1 - hyp * math.cos(sigma2)

                #xTo, yTo = xTo1, yTo1
            
            #self.b_team.robot(0).autoKick = 0.0
            #distanceToChangeState = abs(x1 * (yR - y0) - y1 * (xR - x0) + xR * y0 - yR * x0) / math.sqrt((yR - y0)**2 + (xR - x0)**2)

            #if abs(x1 - xTo) < 130 and abs(y1 - yTo) < 80 and self.state == 0:
            if self.state == 0:
                distanceToChangeState = abs(x1 * (self.yR - y0) - y1 * (self.xR - x0) + self.xR * y0 - self.yR * x0) / math.sqrt((self.yR - y0)**2 + (self.xR - x0)**2)
                #print(distanceToChangeState)
                if distanceToChangeState < 45 and (x1 - xTo) < 130:
                    self.state = 1
                    self.b_team.robot(0).autoKick = 1.0

            #print(self.state, self.yR, xTo, yTo, sep = "    ")
            #if self.state == 0:

            if self.state == 1:
                #self.b_team.robot(0).autoKick = 1.0
                if not self.wasBallKicked:
                    self.firstBallX = x0
                    self.wasBallKicked = True
                
                xTo = self.firstBallX + (R + r + 140)
                yTo = k * xTo + b
                if x1 - self.firstBallX > 50 or abs(self.firstBallX - x0) > 250:
                    self.b_team.robot(0).autoKick = 0.0
                    self.firstBallX = -6000
                    self.wasBallKicked = False
                    self.state = 0

            #print(xTo, yTo);
            #self.b_team.robot(0).go_to_point(auxiliary.Point(xTo, yTo))
            xLast = 0
            yLast = 0
            points = []
            points.append([x1, y1])
            path = []
            way = []
            dists_ = [0 for i in range(len(xRobots))]
            #for i in range(len(xRobots)):
                #way[i] = [0, 0]
                #dists_[i] = 0
            #dists_.append(0)
            counter = 0
            current = -1
            minDists = 10000
            while len(points) != 0:
                wasIntersection = False
                this_point = points[0]
                xNow = this_point[0]
                yNow = this_point[1]
                points.pop(0)

                for i in range(len(xRobots)):
                    if xNow == xRobots[i] and yNow == yRobots[i]: current = i

                for i in range(len(xRobots)):
                    if (not(xNow == xRobots[i] and yNow == yRobots[i]) and self.intersection(xRobots[i], yRobots[i], xNow, yNow, xLast, yLast)):
                        wasIntersection = True
                        dist = math.sqrt((xNow - xRobots[i])**2 + (yNow - yRobots[i])**2)
                        alpha = math.asin(R / dist)
                        kas = math.sqrt(dist**2 - R**2)
                        microAlpha = math.asin((yRobots[i] - yNow) / dist) - alpha
                        point1 = [x0 + kas * math.cos(microAlpha), y0 + kas * math.sin(microAlpha)]
                        microAlpha2 = 2 * alpha + microAlpha
                        point2 = [0 + kas * math.cos(microAlpha2), y0 + kas * math.sin(microAlpha2)]
                        
                        way[i] = this_point
                        if current != -1: 
                            dists_[i] = dists_[current] + 1
                        else:
                            dists_[i] = 1
                        points.append(point1)
                        points.append(point2)
                
                if not wasIntersection:
                    if minDists > dists_[current]: 
                        minDists = dists_[current]
                        path.clear()
                        path.append([xLast, yLast])
                        if current != -1:
                            path.append([xRobots[current], yRobots[current]])
                            iCurrent = current
                            while True:
                                p = way[iCurrent]
                                path.append(p)
                                startPoint = True
                                for i in range(len(xRobots)):
                                    if p[0] == xRobots[i] and p[1] == yRobots[i]: 
                                        iCurrent = i
                                        startPoint = False
                                if startPoint: 
                                    path.append([x1, y1])
                                    break
                        else: path.append([x1, y1])
                    counter += 1
                
                if counter >= 5:
                    break
            
            xTo = path[len(path) - 2][0]
            yTo = path[len(path) - 2][1]

            self.b_team.robot(0).go_to_point(auxiliary.Point(xTo, yTo))
            #self.b_team.robot(0).go_to_point(auxiliary.Point(0, 0))
            #self.b_team.robot(0).go_to_point(auxiliary.Point(0, 0))
            #print(self.state, self.dec)
            #print(x1, y1, xTo, yTo)
            print("GO")
            #self.b_team.robot(0).go_to_point(auxiliary.Point(0, 0))

            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if self.b_team.robot(i).isUsed:
                    rules.append(0)
                    rules.append(self.b_team.robot(i).speedX)
                    rules.append(self.b_team.robot(i).speedY)
                    rules.append(self.b_team.robot(i).speedR)
                    rules.append(self.b_team.robot(i).kickForward)
                    rules.append(self.b_team.robot(i).kickUp)
                    rules.append(self.b_team.robot(i).autoKick)
                    rules.append(self.b_team.robot(i).kickerVoltage)
                    rules.append(self.b_team.robot(i).dribblerEnable)
                    rules.append(self.b_team.robot(i).speedDribbler)
                    rules.append(self.b_team.robot(i).kickerChargeEnable)
                    rules.append(self.b_team.robot(i).beep)            
                    rules.append(0)
                else:
                    for _ in range(0, 13):
                        rules.append(0.0)    
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if self.y_team.robot(i).isUsed:
                    rules.append(0)
                    rules.append(self.y_team.robot(i).speedX)
                    rules.append(self.y_team.robot(i).speedY)
                    rules.append(self.y_team.robot(i).speedR)
                    rules.append(self.y_team.robot(i).kickForward)
                    rules.append(self.y_team.robot(i).kickUp)
                    rules.append(self.y_team.robot(i).autoKick)
                    rules.append(self.y_team.robot(i).kickerVoltage)
                    rules.append(self.y_team.robot(i).dribblerEnable)
                    rules.append(self.y_team.robot(i).speedDribbler)
                    rules.append(self.y_team.robot(i).kickerChargeEnable)
                    rules.append(self.y_team.robot(i).beep)            
                    rules.append(0)
                else:
                    for _ in range(0, 13):
                        rules.append(0.0)    
                     
            self.dt = time.time() - self.cur_time
            self.cur_time = time.time()
            b = bytes()
            rules = b.join((struct.pack('d', rule) for rule in rules))
            self.commands_writer.write(rules)
        # except: None
        from datetime import datetime
        time1 = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        with open("tmp/matlab_controller.txt", "a") as f:
            f.write(time1 + "\n")
