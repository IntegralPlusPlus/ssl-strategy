import math

R = 3
deltaR = 0.07 * R
x1 = 0
y1 = 0
xNow = 0
yNow = 0
xRobots = [-5, -12]
yRobots = [-3, -1]
xLast = -15
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
        
        return distPointLine < R - deltaR
    else: return False
    
def printLine(point1, point2):
    k = (point1[1] - point2[1]) / (point1[0] - point2[0])
    b =  point1[1] - point1[0] * k
    print("y = ", k, " * x + ", b, sep = '')

def printPoint(point):
    print("(", point[0], ", ", point[1], ")", sep = '')

points = []
robotPointsIndx = []
points.append([x1, y1])
robotPointsIndx.append(-1)
path = []
way = []
used = []
dists_ = []
#dists_ = [0 for i in range(len(xRobots))]
for i in range(len(xRobots)):
    dists_.append(0)
    way.append([0, 0])
    used.append(False)
                
counter = 0
current = -1
minDists = 10000
xTo = 0
yTo = 0

while len(points) != 0:
    wasIntersection = False
    inx = robotPointsIndx[0]
                
    if inx != -1: this_robot = [xRobots[inx], yRobots[inx]]
    else: this_robot = [x1, y1]

    print("this_point: ")
    this_point = points[0]
    printPoint(this_point)
    xNow = this_point[0]
    yNow = this_point[1]

    points.pop(0)
    robotPointsIndx.pop(0)

    for i in range(len(xRobots)):
        if this_robot[0] == xRobots[i] and this_robot[1] == yRobots[i]: current = i

    for i in range(len(xRobots)):
        #not(this_robot[0] == xRobots[i] and this_robot[1] == yRobots[i]) and 
        #if not(this_robot[0] == xRobots[i] and this_robot[1] == yRobots[i]) and intersection(xRobots[i], yRobots[i], xNow, yNow, xLast, yLast): wasIntersection = True
        if intersection(xRobots[i], yRobots[i], xNow, yNow, xLast, yLast): wasIntersection = True
        
        if not(this_robot[0] == xRobots[i] and this_robot[1] == yRobots[i]) and not(used[i]) and intersection(xRobots[i], yRobots[i], xNow, yNow, xLast, yLast):
            used[i] = True
            wasIntersection = True
            dist = math.sqrt((xNow - xRobots[i])**2 + (yNow - yRobots[i])**2)
            if R / dist < 1 and R / dist > -1: alpha = math.asin(R / dist)
            elif R / dist < 0: alpha = math.asin(-1)
            else: alpha = math.asin(1)
            
            kas = dist * math.cos(alpha)
                                        
            gamma = math.asin(abs(yRobots[i] - yNow) / dist)
            point1 = [xNow + sign(xRobots[i] - xNow) * kas * math.cos(gamma + alpha), yNow + sign(yRobots[i] - yNow) * kas * math.sin(gamma + alpha)]
            point2 = [xNow + sign(xRobots[i] - xNow) * kas * math.cos(gamma - alpha), yNow + sign(yRobots[i] - yNow) * kas * math.sin(gamma - alpha)]
            
            print("FROM: ", end = "")
            printPoint([xNow, yNow])
            printPoint(point1)
            printPoint(point2)
            
            way[i] = [xNow, yNow]
            if current != -1:  dists_[i] = dists_[current] + 1
            else: dists_[i] = 1
            
            robotPointsIndx.append(i)
            robotPointsIndx.append(i)
            print("Append")
            points.append(point1)
            print("Append")
            points.append(point2)
            #break
        
    if not wasIntersection and current != -1:
        print("PATH, from: ", end = "")
        printPoint(this_point)
        #minDists = 10000
        #print("DISTS_", dists_[current])
        if minDists > dists_[current] and dists_[current] != 0: 
            minDists = dists_[current]
            usedNew = [False for i in range(len(xRobots))]
            path.clear()
            path.append([xLast, yLast])
            path.append([this_point[0], this_point[1]])
            iCurrent = current
    
            while True:
                #print(iCurrent)
                p = way[iCurrent]
                path.append(p)
                if not(p[0] == x1 and p[1] == y1): 
                    #startPoint = True
                    for i in range(len(xRobots)):
                        if not(usedNew[i]) and math.sqrt((p[0] - xRobots[i])**2 + (p[1] - yRobots[i])**2) < R + deltaR: 
                            usedNew[i] = True
                            iCurrent = i
                            break
                else: break
                
            counter += 1
            
for i in range(1, len(path)):
    printLine(path[i], path[i - 1])
    #printPoint(path[i])
