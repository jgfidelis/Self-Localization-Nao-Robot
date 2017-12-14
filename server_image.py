#!/usr/bin/python
from cv2 import sqrt
from scipy import misc
from scipy import ndimage
import numpy as np
import matplotlib.pyplot as plt
import cv2
import socket
import pickle
import struct
import operator
import math
import thread

from ParticleFilter import ParticleFilter
from odometryData import OdometryData

diferencaGols = 0
mediaGols = 0
dy = 0

RIGHT_GOAL_LEFT_POST_X = 4.5
RIGHT_GOAL_LEFT_POST_Z = -0.8
RIGHT_GOAL_RIGHT_POST_X = 4.5
RIGHT_GOAL_RIGHT_POST_Z = 0.8

LEFT_GOAL_LEFT_POST_X = -4.5
LEFT_GOAL_LEFT_POST_Z = 0.8
LEFT_GOAL_RIGHT_POST_X = -4.5
LEFT_GOAL_RIGHT_POST_Z = -0.8

Y_GOAL = 0.85
Y_NAO_HEAD = 0.5
Y_CORNER = 0

# CORNER_TOP_LEFT_X = 4.5
# CORNER_TOP_LEFT_Z = -3.0

CORNERS_X = [4.5, -4.5]
CORNERS_Z = [3.0, -3.0]

useLandmarkValidation = True

def getDistanceBetweenPoints3d(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1))

def getBestLeftPostForDist(x, z, distObserved):
    distToLeftPostLeftGoal = getDistanceBetweenPoints3d(x, Y_NAO_HEAD, z, LEFT_GOAL_LEFT_POST_X, Y_GOAL, LEFT_GOAL_LEFT_POST_Z)
    distToLeftPostRightGoal = getDistanceBetweenPoints3d(x, Y_NAO_HEAD, z, RIGHT_GOAL_LEFT_POST_X, Y_GOAL, RIGHT_GOAL_LEFT_POST_Z)
    deltaToLeftGoal = math.fabs(distObserved - distToLeftPostLeftGoal)
    deltaToRightGoal = math.fabs(distObserved - distToLeftPostRightGoal)
    if (deltaToRightGoal > deltaToLeftGoal):
        return LEFT_GOAL_LEFT_POST_X, LEFT_GOAL_LEFT_POST_Z
    else:
        return RIGHT_GOAL_LEFT_POST_X, RIGHT_GOAL_LEFT_POST_Z

def getBestRightPostForDist(x, z, distObserved):
    distToLeftPostLeftGoal = getDistanceBetweenPoints3d(x, Y_NAO_HEAD, z, LEFT_GOAL_RIGHT_POST_X, Y_GOAL, LEFT_GOAL_RIGHT_POST_Z)
    distToLeftPostRightGoal = getDistanceBetweenPoints3d(x, Y_NAO_HEAD, z, RIGHT_GOAL_RIGHT_POST_X, Y_GOAL, RIGHT_GOAL_RIGHT_POST_Z)
    deltaToLeftGoal = math.fabs(distObserved - distToLeftPostLeftGoal)
    deltaToRightGoal = math.fabs(distObserved - distToLeftPostRightGoal)
    if (deltaToRightGoal > deltaToLeftGoal):
        return LEFT_GOAL_RIGHT_POST_X, LEFT_GOAL_RIGHT_POST_Z
    else:
        return RIGHT_GOAL_RIGHT_POST_X, RIGHT_GOAL_RIGHT_POST_Z

def getBestDistExpectedForCorner(xr, zr, distObserved):

    diffToFoundCorner = 10000.0
    xToUse = 0
    zToUse = 0
    for x in CORNERS_X:
        for z in CORNERS_Z:
            # distance = getDistanceBetweenPoints(flat(gps[0]), float(gps[2]), x, z)
            distanceCalcToCorner = getDistanceBetweenPoints3d(xr, Y_NAO_HEAD, zr, x, Y_CORNER, z)
            deltaDistance = math.fabs(distanceCalcToCorner - distObserved)
            if deltaDistance < diffToFoundCorner:
                diffToFoundCorner = deltaDistance
                xToUse = x
                zToUse = z
    return x, z

def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return
def seg_intersect(a1,a2, b1,b2) :
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )
    return (num / denom.astype(float))*db + b1


def distancex(x1, x2):
    # x1 = np.cos(line1[0][1]) * line1[0][0]
    # x2 = np.cos(line2[0][1]) * line2[0][0]
    return abs(x1[0]-x2[0])

def printLine(image, line,color):
    rho, theta = line
    # print theta
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    cv2.line(image, (x1, y1), (x2, y2), color, 2)

def findLeftVerticalPoleUpperBound(rawImage, edgeMap, lineList):
    if len(lineList) > 1:
        x0 = lineList[0][0]
        x1 = lineList[-1][0]
        for y in range(0, edgeMap.shape[0], 2):
            if(x0-20 > 0 and x1 + 20 < edgeMap.shape[1]):
                for x in range(int(x0-20),int(x1 + 20), 2):
                    if edgeMap[y][x] > 0:
                        cv2.circle(rawImage, (x, y), 5, (0, 0, 255), -1)
                        return x, y
    return 0, 0

def findLeftVerticalPoleLowerBound(rawImage, edgeMap, lineList):
    if len(lineList) > 1:
        x0 = lineList[0][0]
        x1 = lineList[-1][0]
        for y in range(edgeMap.shape[0]-100, -1, -2):
            if (x0 - 20 > 0 and x1 + 20 < edgeMap.shape[1]):
                for x in range(int(x0-20),int(x1 + 20), 2):
                    if edgeMap[y][x] > 0:
                        cv2.circle(rawImage, (x, y), 5, (0, 0, 255), -1)
                        return x, y
    return 0, 0
def findRightVerticalPoleUpperBound(rawImage, edgeMap, lineList):
    if len(lineList) > 1:
        x0 = lineList[0][0]
        x1 = lineList[-1][0]

        if(x0-20 > 0 and x1 < edgeMap.shape[1]):
             for x in range(int(x1), int(x0-20), -2):
                for y in range(0, edgeMap.shape[0], 2):
                    if edgeMap[y][x] > 0:
                        cv2.circle(rawImage, (x, y), 5, (0, 255, 0), -1)
                        return x, y
    return 0, 0

def findRightVerticalPoleLowerBound(rawImage, edgeMap, lineList):
    if len(lineList) > 1:
        x0 = lineList[0][0]
        x1 = lineList[-1][0]
        for y in range(edgeMap.shape[0]-100, -1, -2):
            if (x0 - 20 > 0 and x1 + 20 < edgeMap.shape[1]):
                for x in range(int(x1 + 20), int(x0 - 20), -2):
                    if edgeMap[y][x] > 0:
                        cv2.circle(rawImage, (x, y), 5, (0, 255, 0), -1)
                        return x, y
    return 0,0


def printImage(image, gps):
    file = open('outputlog', 'a')
    fileBola = open('outputlogball', 'a')
    fileCorner = open('outputlogcorner', 'a')
    leftPoleX = 0
    leftPoleY = 0
    rightPoleX = 0
    rightPoleY = 0
    lowLeftPoleY = 0
    lowLeftPoleX = 0
    lowRightPoleX = 0
    lowRightPoleY = 0
    raioBola = 0
    bolaX= 0
    bolaY = 0
    ballDetected = False
    image = cv2.resize(np.float32(image), (600, 800))
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    imageHSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    dst = cv2.transpose(imageHSV)
    #campo
    imgCampo = cv2.inRange(dst, np.array([140,0,0]), np.array([160,1,255]))
    #linhas campo
    imgCampoLinha = cv2.inRange(dst, np.array([120,0,170]), np.array([150,3,255]))
    # imgCampoLinha = cv2.GaussianBlur(imgCampoLinha, (23, 23), 2)
    kernel = np.matrix('0 0 0 0 0; 0 0 0 0 0; 1 1 1 1 1 ; 0 0 0 0 0 ; 0 0 0 0 0').astype(np.uint8);
    # cv2.imshow('frameraw', imgCampoLinha)
    imgCampoLinha = cv2.dilate(imgCampoLinha, kernel, iterations=6)
    kernel = np.ones((5, 5), np.uint8)
    imgCampoLinha = cv2.erode(imgCampoLinha, kernel, iterations=1)
    kernel = np.ones((10, 10), np.uint8)
    imgCampoLinha = imgCampoLinha - cv2.erode(imgCampoLinha, kernel, iterations=1)
    # imgCampoLinha = imgCampoLinha - cv2.erode(imgCampoLinha, kernel, iterations=1)
    global diferencaGols
    global mediaGols
    global dy
    global shouldUpdateParticleWeights
    global currentAngle
    # print "Diferenca",
    # print diferencaGols
    # print "Media",
    # print mediaGols
    # print "dy",
    # print dy
    #branco
    img = cv2.inRange(dst, np.array([0,0,200]), np.array([0,0,255]))
    kernel2 = np.ones((8, 8), np.uint8)
    # img = cv2.erode(img, kernel2, iterations=1)
    # cv2.imshow('frameraw2', img)
    img = cv2.GaussianBlur(img, (23, 23), 3)
    img = cv2.Canny(np.uint8(img), 100, 50);
    imgCampoSemBlur = cv2.Canny(np.uint8(imgCampo), 100, 100)
    # kernel = np.ones((5, 5), np.uint8)
    # imgCampo = cv2.dilate(imgCampo, kernel, iterations=4)
    imgCampo = cv2.GaussianBlur(imgCampo, (23, 23), 4)
    imgCampo = cv2.Canny(np.uint8(imgCampo), 100, 100)
    dst = cv2.transpose(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    verticalCount = 0
    lines = cv2.HoughLines(img, 1, np.pi/2, 60)
    verticalLines = []
    if lines is not None:
        maxLine = None
        for line in lines:
            for rho, theta in line:
                # print theta
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                # cv2.line(dst, (x1, y1), (x2, y2), (0, 255, 255), 2)
                if theta > 1.0 and theta < 2.0:
                    # cv2.line(dst, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    if maxLine is None or y1 < maxLine[2]:
                        maxLine = [line, x1, y1]
                elif theta < 0.3:
                    verticalLines.append(line[0])
                    # cv2.line(dst, (x1, y1), (x2, y2), (255, 0, 0), 2)
        distances =[]
        leftGoalLines = []
        rightGoalLines = []
        verticalLines.sort(key=lambda x: x[0])
        for a,b in zip(verticalLines,verticalLines[1:]):
            # print "distancex: ,"
            # print distancex(a, b)
            distances.append((a, b, distancex(a, b)))
        distances.sort(key=lambda x: x[2])
        if len(distances) >0 and distances[-1] is not None:
            for line in verticalLines:
                if line[0] <= distances[-1][0][0]:
                    # printLine(dst, line, (100, 100, 50))
                    leftGoalLines.append(line)
                else:
                    # printLine(dst, line, (200, 200, 50))
                    rightGoalLines.append(line)
            kernel = np.ones((5, 5), np.uint8)
            imgd = cv2.dilate(img, kernel, iterations=2)
            leftPoleX, leftPoleY = findLeftVerticalPoleUpperBound(dst, imgd, leftGoalLines)
            lowLeftPoleX, lowLeftPoleY = findLeftVerticalPoleLowerBound(dst, imgd, leftGoalLines)
            rightPoleX, rightPoleY = findRightVerticalPoleUpperBound(dst, imgd, rightGoalLines)
            lowRightPoleX, lowRightPoleY = findRightVerticalPoleLowerBound(dst, imgd, rightGoalLines)
            diferencaGols =  leftPoleX - rightPoleX
            mediaGols = (leftPoleX + rightPoleX)/2
            if leftPoleY != 0 and rightPoleY != 0:
                dy = float(leftPoleY) / rightPoleY

    if(len(verticalLines) < 4):
        lines = cv2.HoughLines(img, 1, np.pi / 2, 40)
        verticalLines = []
        if lines is not None:
            maxLine = None
            for line in lines:
                for rho, theta in line:
                    # print theta
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    # cv2.line(dst, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    if theta > 1.0 and theta < 2.0:
                        # cv2.line(dst, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        if maxLine is None or y1 < maxLine[2]:
                            maxLine = [line, x1, y1]
                    elif theta < 0.3:
                        verticalLines.append(line[0])
                        # cv2.line(dst, (x1, y1), (x2, y2), (255, 0, 0), 2)
            distances = []
            leftGoalLines = []
            rightGoalLines = []
            verticalLines.sort(key=lambda x: x[0])
            for a, b in zip(verticalLines, verticalLines[1:]):
                # print "distancex: ,"
                # print distancex(a, b)
                distances.append((a, b, distancex(a, b)))
            distances.sort(key=lambda x: x[2])
            if len(distances) > 0 and distances[-1] is not None:
                for line in verticalLines:
                    if line[0] <= distances[-1][0][0]:
                        # printLine(dst, line, (100, 100, 50))
                        leftGoalLines.append(line)
                    else:
                        # printLine(dst, line, (200, 200, 50))
                        rightGoalLines.append(line)
                kernel = np.ones((5, 5), np.uint8)
                imgd = cv2.dilate(img, kernel, iterations=3)
                leftPoleX, leftPoleY = findLeftVerticalPoleUpperBound(dst, imgd, leftGoalLines)
                lowLeftPoleX, lowLeftPoleY = findLeftVerticalPoleLowerBound(dst, imgd, leftGoalLines)
                rightPoleX, rightPoleY = findRightVerticalPoleUpperBound(dst, imgd, rightGoalLines)
                lowRightPoleX, lowRightPoleY = findRightVerticalPoleLowerBound(dst, imgd, rightGoalLines)
                diferencaGols =  leftPoleX - rightPoleX
                mediaGols = (leftPoleX + rightPoleX) / 2
                if leftPoleY != 0 and rightPoleY != 0 :
                    dy = float(leftPoleY) / rightPoleY

    if (len(verticalLines) < 4):
        lines = cv2.HoughLines(img, 1, np.pi / 2, 20)
        verticalLines = []
        if lines is not None:
            maxLine = None
            for line in lines:
                for rho, theta in line:
                    # print theta
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    # cv2.line(dst, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    if theta > 1.0 and theta < 2.0:
                        # cv2.line(dst, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        if maxLine is None or y1 < maxLine[2]:
                            maxLine = [line, x1, y1]
                    elif theta < 0.3:
                        verticalLines.append(line[0])
                        # cv2.line(dst, (x1, y1), (x2, y2), (255, 0, 0), 2)
            distances = []
            leftGoalLines = []
            rightGoalLines = []
            verticalLines.sort(key=lambda x: x[0])
            for a, b in zip(verticalLines, verticalLines[1:]):
                # print "distancex: ,"
                # print distancex(a, b)
                distances.append((a, b, distancex(a, b)))
            distances.sort(key=lambda x: x[2])
            if len(distances) > 0 and distances[-1] is not None:
                for line in verticalLines:
                    if line[0] <= distances[-1][0][0]:
                        # printLine(dst, line, (100, 100, 50))
                        leftGoalLines.append(line)
                    else:
                        # printLine(dst, line, (200, 200, 50))
                        rightGoalLines.append(line)
                kernel = np.ones((5, 5), np.uint8)
                img = cv2.dilate(img, kernel, iterations=2)
                leftPoleX, leftPoleY = findLeftVerticalPoleUpperBound(dst, img, leftGoalLines)
                lowLeftPoleX, lowLeftPoleY = findLeftVerticalPoleLowerBound(dst, img, leftGoalLines)
                rightPoleX, rightPoleY = findRightVerticalPoleUpperBound(dst, img, rightGoalLines)
                lowRightPoleX, lowRightPoleY = findRightVerticalPoleLowerBound(dst, img, rightGoalLines)
                diferencaGols =  leftPoleX - rightPoleX
                mediaGols = (leftPoleX + rightPoleX) / 2
                if leftPoleY != 0 and rightPoleY != 0 :
                    dy = float(leftPoleY) / rightPoleY


    # img = np.uint8(cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY))
    circles = cv2.HoughCircles(imgCampo, cv2.HOUGH_GRADIENT, 1, 50, param1=1, param2=20, minRadius=5, maxRadius=60)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            bolaX = i[0]
            bolaY = i[1]
            raioBola = i[2]
            seg = dst[ bolaY-raioBola : bolaY+raioBola, bolaX-raioBola : bolaX+raioBola ,0]
            hist = np.histogram(seg, 256, density=True)[0][0]
            # print "histogram"
            # print hist

            if hist > 0.03:
                ballDetected = True
                cv2.circle(dst, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.imshow('frame3', seg / 256)
            # draw the center of the circle
            cv2.circle(dst, (i[0], i[1]), 2, (0, 0, 255), -1)
    # cv2.imshow('frame', img)
    kernel = np.ones((5, 5), np.uint8)
    img_dilation = cv2.dilate(imgCampo, kernel, iterations=2)
    cntsBola = cv2.findContours(img_dilation, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(dst, cntsBola[1], -1, (0, 255, 0), 2)
    kernel = np.ones((5, 5), np.uint8)
    img_dilation = cv2.dilate(imgCampoLinha, kernel, iterations=3)
    lines = cv2.HoughLines(imgCampoLinha, 1, np.pi / 180, 150)

    cornerDetected = False
    cornerX = 0
    cornerY = 0
    detectedLines = []
    if lines is not None:
        lines = sorted(lines, key=lambda x: x[0][0])
        lastAngle = 0
        for line in lines:
            for rho, theta in line:
                # print theta
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                if abs(rho - lastAngle) > 120:
                    cv2.line(dst, (x1, y1), (x2, y2), (0, 255, 255), 3)
                    lastAngle = rho
                    detectedLines.append(([x1, y1], [x2, y2]))
        for a, b in zip(detectedLines, detectedLines[1:]):
            # print a[0][0]
            p1 = np.array(a[0])
            p2 = np.array(a[1])

            p3 = np.array(b[0])
            p4 = np.array(b[1])

            x, y =  seg_intersect(p1, p2, p3, p4)
            cornerX = x
            cornerY = y
            cornerDetected = True
            try:
                cv2.circle(dst, (int(x),int(y)), 2, (0, 0, 255),2)
            except:
                pass

    leftPoleLength = math.sqrt(pow((leftPoleX - lowLeftPoleX), 2) + pow((leftPoleY - lowLeftPoleY), 2))
    rightPoleLength = math.sqrt(pow((rightPoleX - lowRightPoleX), 2) + pow((rightPoleY - lowRightPoleY), 2))
    topPoleLength = math.sqrt(pow((rightPoleX - leftPoleX), 2) + pow((rightPoleY - leftPoleY), 2))

    # print "left pole: ",
    # print leftPoleLength
    # print "right pole: ",
    # print rightPoleLength
    # print "top pole: ",
    # print topPoleLength
    # gol tem 0.85 metros
    # distancia focal e 680 ou 720
    # raio bola 0.067 m

    landmarkList = []

    try:
        directionAngleLeft = (abs((float(leftPoleX)/800) - 0.5)) * 1.064
        c = math.cos(directionAngleLeft)
        distLeftPole = ((720 * 0.85) / leftPoleLength) / c
        # print "distancia gol",
        # file.write("gps: ")
        # file.write(str(gps[0])+"," +str(gps[1])+","+str(gps[2])+"\n")
        # file.write("dist left pole: ")
        # file.write("angulo: ")
        # file.write(str(math.degrees(directionAngle))+"\n")
        # file.write(str(distLeftPole) + "\n")
        cv2.putText(dst, str(distLeftPole)+"m", (leftPoleX, leftPoleY),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)
        directionAngleRight = (abs((float(rightPoleX) / 800) - 0.5)) * 1.064
        c = math.cos(directionAngleRight)
        distRightPole = ((720 * 0.85) / rightPoleLength) /c
        # print "distancia gol",
        # print distLeftPole
        cv2.putText(dst, str(distRightPole) + "m", (rightPoleX, rightPoleY),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        poleRatio = max([leftPoleLength, rightPoleLength]) / min([leftPoleLength, rightPoleLength])
        # goalRatio = (1.6 / 0.85) / poleRatio
        goalRatio = (1.6 / 0.85)
        value = (goalRatio/2*(leftPoleLength + rightPoleLength) - topPoleLength) / (goalRatio/2*(leftPoleLength + rightPoleLength) +  topPoleLength)
        c = 0.6
        exp = -pow((value/c), 2)/2
        o = pow(math.e, exp)

        # print "o: ",
        # print o

        if o > 0.5:
            file.write("gps: ")
            file.write(str(gps[0]) + "," + str(gps[1]) + "," + str(gps[2]) + "\n")
            file.write("dist left pole: ")
            file.write(str(distLeftPole) + "\n")
            file.write("angulo: ")
            file.write(str(math.degrees(directionAngleLeft)) + "\n")
            file.write("dist right pole: ")
            file.write(str(distRightPole) + "\n")
            file.write("angulo: ")
            file.write(str(math.degrees(directionAngleRight)) + "\n")
            # gol detectado
            cv2.line(dst, (leftPoleX, leftPoleY), (rightPoleX, rightPoleY), (20,100,150))
            cv2.line(dst, (leftPoleX, leftPoleY), (lowLeftPoleX, lowLeftPoleY), (20, 100, 150))
            cv2.line(dst, (rightPoleX, rightPoleY), (lowRightPoleX, lowRightPoleY), (20, 100, 150))
            cv2.circle(dst, (((leftPoleX+rightPoleX)/2), ((leftPoleY+lowLeftPoleY)/2)), 3, (20,100,150))

            if useLandmarkValidation is True:
                if (particleFilter.isLandmarkValid(float(gps[0]), float(gps[2]), distLeftPole, 0) is True):
                    # landmarkList.append([distLeftPole, directionAngleLeft, 0])
                    x, z = getBestLeftPostForDist(float(gps[0]), float(gps[2]), distLeftPole)
                    landmarkList.append([distLeftPole, directionAngleLeft, 0, x, z])
                if (particleFilter.isLandmarkValid(float(gps[0]), float(gps[2]), distRightPole, 1) is True):
                    # landmarkList.append([distRightPole, directionAngleRight, 1])
                    x, z = getBestRightPostForDist(float(gps[0]), float(gps[2]), distRightPole)
                    landmarkList.append([distRightPole, directionAngleRight, 1, x, z])
            else:
                # landmarkList.append([distLeftPole, directionAngleLeft, 0])
                x, z = getBestLeftPostForDist(float(gps[0]), float(gps[2]), distLeftPole)
                landmarkList.append([distLeftPole, directionAngleLeft, 0, x, z])
                # landmarkList.append([distRightPole, directionAngleRight, 1])
                x, z = getBestRightPostForDist(float(gps[0]), float(gps[2]), distRightPole)
                landmarkList.append([distRightPole, directionAngleRight, 1, x, z])
            # if (math.fabs(distLeftPole - distToRightGoalLeftPost) < math.fabs(distLeftPole - distToLeftGoalLeftPost)):
            #     #se a distancia calculada eh muito pior que a melhor distancia, tem algo errado... nao da pq
            #     landmarkList.append([RIGHT_GOAL_LEFT_POST_X, RIGHT_GOAL_LEFT_POST_Z, distLeftPole, math.degrees(directionAngleLeft)])
            #     landmarkList.append([RIGHT_GOAL_RIGHT_POST_X, RIGHT_GOAL_RIGHT_POST_Z, distRightPole, math.degrees(directionAngleRight)])
            # else:
            #     landmarkList.append([LEFT_GOAL_LEFT_POST_X, LEFT_GOAL_LEFT_POST_Z, distLeftPole, math.degrees(directionAngleLeft)])
            #     landmarkList.append([LEFT_GOAL_RIGHT_POST_X, LEFT_GOAL_RIGHT_POST_Z, distRightPole, math.degrees(directionAngleRight)])
            #             xLandmark = listOfObservations[i][0]
            # zLandmark = listOfObservations[i][1]
            # distObs = listOfObservations[i][2] distLeftPole
            # angleObs = listOfObservations[i][3] math.degrees(directionAngleLeft))
            # landmarkList.append([])
    # except Exception, e:
        # print("Error:" + str(e))
    except:
        pass
    if ballDetected:
        try:
            fileBola.write("gps: ")
            fileBola.write(str(gps[0]) + "," + str(gps[1]) + "," + str(gps[2]) + "\n")
            directionAngle = (abs((float(bolaX) / 800) - 0.5)) * 1.064
            elevationAngle= (abs((float(bolaY + raioBola) / 600) - 0.5)) * 0.83
            fileBola.write("angulo: ")
            fileBola.write(str(math.degrees(directionAngle)) + "\n")
            c1 = math.cos(directionAngle)
            c2 = math.cos(elevationAngle)
            distBola = ((720 * 0.067) / raioBola) / (c1*c2)
            distBola = math.sqrt(math.pow(distBola,2) - math.pow(0.51, 2))
            distBola = math.tan((math.pi/2) - elevationAngle) * 0.51 / c1
            fileBola.write("distBola:")
            fileBola.write(str(distBola) +"\n")
            cv2.putText(dst, str(distBola) + "m", (bolaX, bolaY),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
        except:
            pass

    if cornerDetected:
        try:
            fileCorner.write("gps: ")
            fileCorner.write(str(gps[0]) + "," + str(gps[1]) + "," + str(gps[2]) + "\n")
            directionAngle = (abs((float(cornerX) / 800) - 0.5)) * 1.064
            elevationAngle= (abs((float(cornerY) / 600) - 0.5)) * 0.83
            fileCorner.write("angulo: ")
            fileCorner.write(str(math.degrees(directionAngle)) + "\n")
            c1 = math.cos(directionAngle)
            c2 = math.cos(elevationAngle)
            # distBola = ((720 * 0.067) / raioBola) / (c1*c2)
            # distBola = math.sqrt(math.pow(distBola,2) - math.pow(0.51, 2))
            distCorner = math.tan((math.pi/2) - elevationAngle) * 0.51 / c1
            fileCorner.write("distCorner:")
            fileCorner.write(str(distCorner) +"\n")
            #procurar corner com d mais perto da distancia que acharmos
            if useLandmarkValidation is True:
                if (particleFilter.isLandmarkValid(float(gps[0]), float(gps[2]), distCorner, 2) is True):
                    # landmarkList.append([distCorner, directionAngle, 2])
                    x, z = getBestDistExpectedForCorner(float(gps[0]), float(gps[2]), distCorner)
                    landmarkList.append([distCorner, directionAngle, 2, x, z])
            else:
                # landmarkList.append([distCorner, directionAngle, 2])
                x, z = getBestDistExpectedForCorner(float(gps[0]), float(gps[2]), distCorner)
                landmarkList.append([distCorner, directionAngle, 2, x, z])
            # diffToFoundCorner = 10000.0
            # xFound = -1
            # zFound = -1
            # for x in CORNERS_X:
            #     for z in CORNERS_Z:
            #         # distance = getDistanceBetweenPoints(flat(gps[0]), float(gps[2]), x, z)
            #         distance = getDistanceBetweenPoints3d(flat(gps[0]), Y_NAO_HEAD, float(gps[2]), x, Y_CORNER, z)
            #         if math.fabs(distance - distCorner) < diffToFoundCorner:
            #             xFound = x
            #             zFound = z
            # landmarkList.append([xFound, zFound, distCorner, math.degrees(directionAngle)])



        except:
            pass



    # cv2.imshow('frame3', imgCampo)
    # cv2.imshow('frame4', imgCampoLinha)
    # print("Chegou aqui")
    # print ("Tamanho do vetor " + str(len(landmarkList)))
    # if len(landmarkList) > 0 and shouldUpdateParticleWeights is True:
    if len(landmarkList) > 0:
        shouldUpdateParticleWeights = False
        particleFilter.calculaleWeightWithLandmarkCoordinates(landmarkList)
        # particleFilter.calculaleWeightWithLandmarkCoordinatesWithCorrectCoordinates(landmarkList)
        # particleFilter.getWeightedMeanPosition()
        # landmarkList = []

    cv2.imshow('frame2', dst/256)
    cv2.waitKey(1)


def printMap(coordinates):
    x = int((coordinates[0]*100 + 450)/2)
    y = int((coordinates[2]*100 + 300)/2)
    # print "x ",
    # print coordinates[0]
    # print y
    img = np.zeros((300, 450, 3), np.uint8)
    img[:, :, 1] = 128
    # middle line
    cv2.line(img, (225, 0), (225, 300), (255, 255, 255), 5)
    # field countours
    cv2.line(img, (0, 0), (450, 0), (255, 255, 255), 5)
    cv2.line(img, (0, 300), (450, 300), (255, 255, 255), 5)
    cv2.line(img, (0, 0), (0, 300), (255, 255, 255), 5)
    cv2.line(img, (450, 0), (450, 300), (255, 255, 255), 5)
    # gol area left
    cv2.line(img, (0, 50), (50, 50), (255, 255, 255), 5)
    cv2.line(img, (0, 250), (50, 250), (255, 255, 255), 5)
    cv2.line(img, (50, 50), (50, 250), (255, 255, 255), 5)
    # penalty circle left
    img = cv2.circle(img, (105, 150), 7, (255, 255, 255), -1)
    # gol area right
    cv2.line(img, (400, 50), (450, 50), (255, 255, 255), 5)
    cv2.line(img, (400, 250), (450, 250), (255, 255, 255), 5)
    cv2.line(img, (400, 50), (400, 250), (255, 255, 255), 5)
    # penalty circle right
    img = cv2.circle(img, (345, 150), 7, (255, 255, 255), -1)
    # center circles
    img = cv2.circle(img, (225, 150), 7, (255, 255, 255), -1)
    img = cv2.circle(img, (225, 150), 38, (255, 255, 255), 3)

    #nao robot
    img = cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
    #nao robot estimated
    xest = 0.5 * abs(diferencaGols) + 95
    yest = 300 - (0.4 * mediaGols )
    img = cv2.circle(img, (int(xest), int(yest)), 5, (255, 0, 0), -1)
    cv2.imshow('map', img)
    cv2.waitKey(1)


def handleConnection(conn, addr):
    while 1:
        rawSize = conn.recv(4)
        size = struct.unpack("i", rawSize)[0]
        readLength = 0
        result = ""
        # while 1:
            # print "Size: " + str(size)
        data = conn.recv(size)
        string = str(pickle.loads(data))
        # print string
        if string == "IMAGE":
            rawSize = conn.recv(4)
            size = struct.unpack("i", rawSize)[0]
            while 1:
                if size - readLength > 100:
                    data = conn.recv(100)
                elif size - readLength == 0:
                    image = np.array(pickle.loads(result))
                    printImage(image, gps)
                    result = ""
                    readLength = 0
                    break
                else:
                    data = conn.recv(size - readLength)
                if not data: break
                readLength += len(data)
                result += data
        elif string == "GPS":
            # print "Received GPS"
            rawSize = conn.recv(4)
            size = struct.unpack("i", rawSize)[0]
            data = conn.recv(size)
            gps = pickle.loads(data)
            particleFilter.robotX = float(gps[0])
            particleFilter.robotZ = float(gps[2])
            printMap(gps)

        elif string == "ODOMETRY":
            rawSize = conn.recv(4)
            size = struct.unpack("i", rawSize)[0]
            data = conn.recv(size)
            odometry = pickle.loads(data)
            odometryData = OdometryData(float(odometry[0]), float(odometry[1]), float(odometry[2]))
            particleFilter.updateParticlesWithOdometry(odometryData)
            shouldUpdateParticleWeights = True
        elif string == "ANGLE":
            rawSize = conn.recv(4)
            size = struct.unpack("i", rawSize)[0]
            data = conn.recv(size)
            currentAngle = pickle.loads(data)
            particleFilter.robotAngle = float(currentAngle)



shouldUpdateParticleWeights = True
HOST = 'localhost'
PORT = 50040
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(2)

particleFilter = ParticleFilter()
gps = []
currentAngle = 0

while 1:
    conn, addr = s.accept()
    print 'Connected by', addr
    thread.start_new_thread(handleConnection,(conn,addr))

conn.close()
# plt.imshow(outer, cmap='gray')
