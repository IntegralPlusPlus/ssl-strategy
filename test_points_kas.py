import math

R = 2
xNow = 0
yNow = 0
xRobots = -4
yRobots = -2
xLast = -6
yLast = -4

def sign(a):
    if a >= 0: return 1
    else: return -1

def between(x, y, xFrom, yFrom, xTo, yTo):
    return ((xFrom - R < x and x < xTo + R) or (xTo - R < x and x < xFrom + R)) \
            and ((yFrom - R < y and y < yTo + R) or (yTo - R < y and y < yFrom + R))
    
def intersection(x, y, xFrom, yFrom, xTo, yTo):
    if between(x, y, xFrom, yFrom, xTo, yTo):
        if xTo != xFrom: a = 1 / (xTo - xFrom)
        else: a = 0
        
        if yTo != yFrom: b = -1 / (yTo - yFrom)
        else: b = 0
        
        c = -yFrom * b - xFrom * a
        distPointLine = abs(a * x + b * y + c) / math.sqrt(a**2 + b**2) 
        
        return distPointLine < R + 20
    else: return False

def printPoint(point):
    print("(", point[0], ", ", point[1], ")", sep = '')
    
if intersection(xRobots, yRobots, xNow, yNow, xLast, yLast):
    dist = math.sqrt((xNow - xRobots)**2 + (yNow - yRobots)**2)
    if R / dist < 1 and R / dist > -1: alpha = math.asin(R / dist)
    elif R / dist < 0: alpha = math.asin(-1)
    else: alpha = math.asin(1)
    
    kas = dist * math.cos(alpha)
                            
    gamma = math.asin(abs(yRobots - yNow) / dist)
    point1 = [xNow + sign(xRobots - xNow) * kas * math.cos(gamma + alpha), yNow + sign(yRobots - yNow) * kas * math.sin(gamma + alpha)]
    point2 = [xNow + sign(xRobots - xNow) * kas * math.cos(gamma - alpha), yNow + sign(yRobots - yNow) * kas * math.sin(gamma - alpha)]
    
    printPoint(point1)
    printPoint(point2)
