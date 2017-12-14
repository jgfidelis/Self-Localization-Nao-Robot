
import numpy as np
from Particle import Particle
from odometryData import OdometryData
MAX_X = 3.001
MIN_X = -3.0
MAX_Z = 4.501
MIN_Z = -4.5
MIN_ANGLE = 0.0
MAX_ANGLE = 360.0
PARTICLE_NUM = 100

import math
import copy
import random

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

CORNERS_X = [4.5, -4.5]
CORNERS_Z = [3.0, -3.0]

# landmark 1 -> left post
# landmark 2 -> right post
#landmark 3 -> corner

def getBestDistExpectedForLeftPost(particle, distObserved):
    distToLeftPostLeftGoal = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, LEFT_GOAL_LEFT_POST_X, Y_GOAL, LEFT_GOAL_LEFT_POST_Z)
    distToLeftPostRightGoal = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, RIGHT_GOAL_LEFT_POST_X, Y_GOAL, RIGHT_GOAL_LEFT_POST_Z)
    angleLeftPostLeftGoal = getAngleBetweenPoints(particle, LEFT_GOAL_LEFT_POST_X, LEFT_GOAL_LEFT_POST_Z)
    angleLeftPostRightGoal = getAngleBetweenPoints(particle, RIGHT_GOAL_LEFT_POST_X, RIGHT_GOAL_LEFT_POST_Z)
    deltaToLeftGoal = math.fabs(distObserved - distToLeftPostLeftGoal)
    deltaToRightGoal = math.fabs(distObserved - distToLeftPostRightGoal)
    if (deltaToRightGoal > deltaToLeftGoal):
        return distToLeftPostLeftGoal, angleLeftPostLeftGoal
    else:
        return distToLeftPostRightGoal, angleLeftPostRightGoal

def getBestDistExpectedForRightPost(particle, distObserved):
    distToRightPostLeftGoal = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, LEFT_GOAL_RIGHT_POST_X, Y_GOAL, LEFT_GOAL_RIGHT_POST_Z)
    distToRightPostRightGoal = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, RIGHT_GOAL_RIGHT_POST_X, Y_GOAL, RIGHT_GOAL_RIGHT_POST_Z)
    angleRightPostLeftGoal = getAngleBetweenPoints(particle, LEFT_GOAL_RIGHT_POST_X, LEFT_GOAL_RIGHT_POST_Z)
    angleRightPostRightGoal = getAngleBetweenPoints(particle, RIGHT_GOAL_RIGHT_POST_X, RIGHT_GOAL_RIGHT_POST_Z)
    deltaToLeftGoal = math.fabs(distObserved - distToRightPostLeftGoal)
    deltaToRightGoal = math.fabs(distObserved - distToRightPostRightGoal)
    if (deltaToRightGoal > deltaToLeftGoal):
        return distToRightPostLeftGoal, angleRightPostLeftGoal
    else:
        return distToRightPostRightGoal, angleRightPostRightGoal

def getBestDistExpectedForCorner(particle, distObserved):

    diffToFoundCorner = 10000.0
    bestDistance = 10000.0
    angleToBestGoal = 0
    for x in CORNERS_X:
        for z in CORNERS_Z:
            # distance = getDistanceBetweenPoints(flat(gps[0]), float(gps[2]), x, z)
            distanceCalcToCorner = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, x, Y_CORNER, z)
            angle = getAngleBetweenPoints(particle, x, z)
            deltaDistance = math.fabs(distanceCalcToCorner - distObserved)
            if deltaDistance < diffToFoundCorner:
                diffToFoundCorner = deltaDistance
                bestDistance = distanceCalcToCorner
                angleToBestGoal = angle
    return bestDistance, angleToBestGoal

def getDistanceBetweenPoints3d(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1))

def getDistanceBetweenPoints(x1, z1, x2, z2):
    return math.hypot(x2-x1, z2-z1)

def getStdDeviationForUniformDistribution(mmax, mmin):
    delta = mmax-mmin
    return math.sqrt(delta*delta/12)

def getAngleBetweenPoints(particle, x2, z2):
    x1 = particle.x
    z1 = particle.z
    particleAngle = particle.angle
    vectorRobot = [math.sin(math.radians(particleAngle)), math.cos(math.radians(particleAngle))]
    vectorDirection = [z2-z1, x2-x1]
    dotProduct = np.dot(vectorRobot, vectorDirection)
    moduleVectorRobot = np.linalg.norm(vectorRobot)
    moduleVectorDirection = np.linalg.norm(vectorDirection)
    cosTeta = dotProduct / (moduleVectorDirection*moduleVectorRobot)
    teta = math.acos(cosTeta)
    return teta

class ParticleFilter():

    def __init__(self):
        self.particles = []
        self.weightsSum = PARTICLE_NUM
        self.aggregatedWeights = []
        self.distDeltaList = []
        self.updates = 0
        self.hasSeenLeftPost = False
        self.hasSeenRightPost = False
        self.hasSeenOneCorner = False
        self.robotX = 0
        self.robotZ = 0
        self.robotAngle = 0
        self.posFile = open("particleFile.csv", 'w')
        self.posFile.write("Update,AvgX,AvgZ,AvgAngle,RobotX,RobotZ,RobotAngle\n")

        for i in range(PARTICLE_NUM):
            particle = self.generateRandomParticle()
            # print("Particle: x: %f, z: %f, angle: %f" % (particle.x,particle.z,particle.angle))
            self.particles.append(particle)
            self.aggregatedWeights.append(i+1)

    def generateRandomParticle(self):
        x = np.random.uniform(MIN_X, MAX_X)
        z = np.random.uniform(MIN_Z, MAX_Z)
        angle = np.random.uniform(MIN_ANGLE, MAX_ANGLE)
        weight = 1.0
        # print("x: %f, z: %f, angle: %f" % (x,z,angle))
        return Particle(x, z, angle, weight)

    def isParticleValid(self, particle):
        if particle.x > MAX_X or particle.x < MIN_X or particle.z > MAX_Z or particle.z < MIN_Z:
            return False
        else:
            return True

    def isLandmarkValid(self, naox, naoz, distObserved, landmarkType):
        naoParticle = Particle(naox, naoz, 0, 0)
        distExpected = 0
        if (landmarkType == 0):
                distExpected, temp = getBestDistExpectedForLeftPost(naoParticle, distObserved)
        elif (landmarkType == 1):
                distExpected, temp = getBestDistExpectedForRightPost(naoParticle, distObserved)
        elif (landmarkType == 2):
                distExpected, temp = getBestDistExpectedForCorner(naoParticle, distObserved)
        if (distExpected > 0):
            relation = distObserved/distExpected
            # print("DistObserved: " + str(distObserved)+", distExpected: "+str(distExpected)+", relation: " + str(relation))
            return relation > 0.85 and relation < 1.15


    def updateParticlesWithOdometry(self, odometryData):
        print("updateParticlesWithOdometry")
        self.resetWeights()
        for particle in self.particles:
            particle.updateParticleWithOdometryData(odometryData)
            if self.isParticleValid(particle) is False:
                # print("Invalid, generating a new one")
                particle = self.generateRandomParticle()

    def resetWeights(self):
        self.hasSeenLeftPost = False
        self.hasSeenRightPost = False
        self.hasSeenOneCorner = False
        for particle in self.particles:
            particle.weight = 1.0

    def calculaleWeightWithLandmarkCoordinates(self, listOfObservations):
        # print("calculaleWeightWithLandmarkCoordinates")
        hasModified = False
        for i in range(len(listOfObservations)):
            distObserved = listOfObservations[i][0]
            angObserved = listOfObservations[i][1]
            landmarkType = listOfObservations[i][2]
            if (landmarkType == 0 and self.hasSeenLeftPost is False):
                self.hasSeenLeftPost = True
                # xToCheck, zToCheck = getCorrectPostFromRandomSampleForLeftPost(self.particles, distObserved)
            elif (landmarkType == 1 and self.hasSeenRightPost is False):
                self.hasSeenRightPost = True
                # xToCheck, zToCheck = getCorrectPostFromRandomSampleForRightPost(self.particles, distObserved)
            elif (landmarkType == 2 and self.hasSeenOneCorner is False):
                self.hasSeenOneCorner = True
                # xToCheck, zToCheck = getCorrectPostFromRandomSampleForCorners(self.particles, distObserved)
            else:
                continue

            hasModified = True
            for particle in self.particles:
                distExpected = 0
                angleExpected = 0
                # self.distDeltaList.append(distDeltaLocal)
                # distAvg = np.mean(self.distDeltaList)
                # prob = math.exp(-(math.fabs(distExpected-distObs)/(2*math.erf(math.fabs(distAvg-distDeltaLocal))))) * math.exp(-(math.fabs(angleExpected-angleObs)/(2*math.erf(bobserved))))
                # prob = math.exp(-(math.fabs(distExpected-distObs)/(2*math.erf(math.fabs(distAvg-distDeltaLocal)))))
                if (landmarkType == 0):
                    #getWhich post to use
                    distExpected, angleExpected = getBestDistExpectedForLeftPost(particle, distObserved)
                    # distExpected = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, xToCheck, Y_GOAL, zToCheck)
                elif (landmarkType == 1):
                    distExpected, angleExpected = getBestDistExpectedForRightPost(particle, distObserved)
                    # distExpected = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, xToCheck, Y_GOAL, zToCheck)
                elif (landmarkType == 2):
                    distExpected, angleExpected = getBestDistExpectedForCorner(particle, distObserved)
                    # distExpected = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, xToCheck, Y_CORNER, zToCheck)
                if (distExpected > 0):
                    # print "ANgle obs: " + str(math.degrees(angObserved))
                    # print "ANgle exp: " + str(math.degrees(angleExpected))
                    # print "Delta: " + str(math.degrees(math.fabs(angleExpected-angObserved)))
                    prob = math.exp(-(math.fabs(distExpected-distObserved)/2)) * math.exp(-(math.fabs(angleExpected-angObserved)/2))
                    particle.weight *= prob


        if (hasModified is False):
            return
        self.weightsSum = 0
        for i in range(PARTICLE_NUM):
            particle = self.particles[i]
            self.weightsSum += particle.weight
            self.aggregatedWeights[i] = self.weightsSum

        self.drawParticles()

    def calculaleWeightWithLandmarkCoordinatesWithCorrectCoordinates(self, listOfObservations):
        # print("calculaleWeightWithLandmarkCoordinates")
        hasModified = False
        for i in range(len(listOfObservations)):
            distObserved = listOfObservations[i][0]
            angObserved = listOfObservations[i][1]
            landmarkType = listOfObservations[i][2]
            xToCheck = listOfObservations[i][3]
            zToCheck = listOfObservations[i][4]
            if (landmarkType == 0 and self.hasSeenLeftPost is False):
                self.hasSeenLeftPost = True
                # xToCheck, zToCheck = getCorrectPostFromRandomSampleForLeftPost(self.particles, distObserved)
            elif (landmarkType == 1 and self.hasSeenRightPost is False):
                self.hasSeenRightPost = True
                # xToCheck, zToCheck = getCorrectPostFromRandomSampleForRightPost(self.particles, distObserved)
            elif (landmarkType == 2 and self.hasSeenOneCorner is False):
                self.hasSeenOneCorner = True
                # xToCheck, zToCheck = getCorrectPostFromRandomSampleForCorners(self.particles, distObserved)
            else:
                continue

            hasModified = True
            for particle in self.particles:
                distExpected = 0
                angleExpected = 0
                if (landmarkType == 0):
                    #getWhich post to use
                    # distExpected, angleExpected = getBestDistExpectedForLeftPost(particle, distObserved)
                    distExpected = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, xToCheck, Y_GOAL, zToCheck)
                    angleExpected = getAngleBetweenPoints(particle, xToCheck, zToCheck)
                elif (landmarkType == 1):
                    # distExpected, angleExpected = getBestDistExpectedForRightPost(particle, distObserved)
                    distExpected = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, xToCheck, Y_GOAL, zToCheck)
                    angleExpected = getAngleBetweenPoints(particle, xToCheck, zToCheck)
                elif (landmarkType == 2):
                    # distExpected, angleExpected = getBestDistExpectedForCorner(particle, distObserved)
                    distExpected = getDistanceBetweenPoints3d(particle.x, Y_NAO_HEAD, particle.z, xToCheck, Y_CORNER, zToCheck)
                    angleExpected = getAngleBetweenPoints(particle, xToCheck, zToCheck)
                if (distExpected > 0):
                    prob = math.exp(-(math.fabs(distExpected-distObserved)/2)) * math.exp(-(math.fabs(angleExpected-angObserved)/2))
                    particle.weight *= prob


        if (hasModified is False):
            return
        self.weightsSum = 0
        for i in range(PARTICLE_NUM):
            particle = self.particles[i]
            self.weightsSum += particle.weight
            self.aggregatedWeights[i] = self.weightsSum

        self.drawParticles()

    def drawParticles(self):
        self.updates += 1
        tempNewParticles = []
        for i in range(PARTICLE_NUM):
            randomNum = np.random.uniform(0, self.weightsSum)
            particleTemp = self.drawParticle(randomNum)
            tempNewParticles.append(particleTemp)

        self.particles = tempNewParticles[:]
        self.getWeightedMeanPosition()


    def drawRandomParticle(self, randomNum, particle):
        if (randomNum <= particle.weight):
            return particle
        else:
            return self.generateRandomParticle()

    def drawParticle(self, randomNum):
        lowIndex = 0
        highIndex = PARTICLE_NUM
        while lowIndex <= highIndex:
            midNum = (lowIndex+highIndex)//2
            rangeLower = 0
            if midNum - 1 >= 0:
                rangeLower = self.aggregatedWeights[midNum-1]

            rangeUpper = self.aggregatedWeights[midNum]
            if randomNum <rangeLower:
                highIndex = midNum-1
            elif randomNum >= rangeUpper:
                lowIndex = midNum+1
            elif randomNum >= rangeLower and randomNum < rangeUpper:
                return copy.deepcopy(self.particles[midNum])

    def getWeightedMeanPosition(self):
        position = []
        x = 0
        z = 0
        angle = 0
        self.weightsSum = 0
        for particle in self.particles:
            self.weightsSum += particle.weight
            x += particle.x*particle.weight
            z += particle.z*particle.weight
            angle += particle.angle*particle.weight

        avgX = x / self.weightsSum
        avgZ = z / self.weightsSum
        avgAngle = angle / self.weightsSum
        position.append(avgX)
        position.append(avgZ)
        position.append(avgAngle)
        print "Updates: " + str(self.updates)
        print "AvgX: " + str(avgX) + " AvgZ: " + str(avgZ) + " AvgAngle: " + str(avgAngle)
        print "RobotX: " + str(self.robotX) +" Robot Z: "+ str(self.robotZ) + " robotAngle: " + str(self.robotAngle)
        self.posFile.write(str(self.updates)+","+str(avgX)+","+str(avgZ)+","+str(avgAngle)+","+str(self.robotX)+","+str(self.robotZ)+","+str(self.robotAngle)+"\n")
        return position


