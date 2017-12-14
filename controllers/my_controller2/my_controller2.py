"""Example of Python controller for Nao robot.
   This demonstrates how to access sensors and actuators"""

from controller import Robot, Accelerometer, Camera, DistanceSensor, \
                       GPS, Gyro, InertialUnit,Keyboard, LED, Motion, \
                       Motor, TouchSensor, PositionSensor
from scipy import misc
from scipy import ndimage
import numpy as np
import matplotlib.pyplot as plt
import cv2
import socket
import pickle
import struct

from odometryData import OdometryData
from BasicOdometryProvider import BasicOdometryProvider
# from ParticleFilter import ParticleFilter

import ctypes

# this is the main class
class Nao (Robot):
    PHALANX_MAX = 8

    # load motion files
    def loadMotionFiles(self):
        self.handWave = Motion('../../motions/HandWave.motion')
        self.forwards = Motion('../../motions/Forwards50.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')

    def startMotion(self, motion):
        # interrupt current motion
        # if self.currentlyPlaying:
        #     self.currentlyPlaying.stop()

        if (self.currentlyPlaying and self.currentlyPlaying != self.turnLeft60 and self.currentlyPlaying != self.turnRight60):
            self.currentlyPlaying.stop()
            self.updateOdometry()
            motion.play()
            self.currentlyPlaying = motion
        else:
            motion.play()
            self.currentlyPlaying = motion
        # start new motion
        # motion.play()
        # self.currentlyPlaying = motion

    # the accelerometer axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print '----------accelerometer----------'
        print 'acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2])

    # the gyro axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printGyro(self):
        vel = self.gyro.getValues()
        print '----------gyro----------'
        # z value is meaningless due to the orientation of the Gyro
        print 'angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1])

    def printGps(self):
        p = self.gps.getValues()
        print '----------gps----------'
        print 'position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2])
        p = self.HeadPitchS.getValue()
        print '----------HeadPitchS----------'
        print 'HeadPitchS : %f' % p
        p = self.RElbowRollS.getValue()
        print '----------RElbowRollS----------'
        print 'RElbowRollS : %f' % p
        p = self.LElbowRollS.getValue()
        a = self.LElbowRollS.getType()
        print '----------LElbowRollS----------'
        print 'LElbowRollS : %f %f' % (p, a)
        p = self.RShoulderPitchS.getValue()
        print '----------RShoulderPitchS----------'
        print 'RShoulderPitchS : %f' % p
        p = self.RAnklePitchS.getValue()
        print '----------RAnklePitchS----------'
        print 'RAnklePitchS : %f' % p


    # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print '----------inertial unit----------'
        print 'roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2])

    def printFootSensors(self):
        newtons = 0.0
        fsv = [] # force sensor values

        fsv.append(self.fsr[0].getValues())
        fsv.append(self.fsr[1].getValues())

        #a = self.getMotor("HeadYaw")

        l = []
        r = []

        newtonsLeft = 0
        newtonsRight = 0

        # The coefficients were calibrated against the real
        # robot so as to obtain realistic sensor values.
        l.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1]) # Left Foot Front Left
        l.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1]) # Left Foot Front Right
        l.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1]) # Left Foot Rear Right
        l.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1]) # Left Foot Rear Left

        r.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1]) # Right Foot Front Left
        r.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1]) # Right Foot Front Right
        r.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1]) # Right Foot Rear Right
        r.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1]) # Right Foot Rear Left

        for i in range(0, len(l)):
            l[i] = max(min(l[i], 25), 0)
            r[i] = max(min(r[i], 25), 0)
            newtonsLeft += l[i]
            newtonsRight += r[i]

        print '----------foot sensors----------'
        print '+ left ---- right +'
        print '+-------+ +-------+'
        print '|'  + str(round(l[0],1)) + \
              '  ' + str(round(l[1],1)) + \
              '| |'+ str(round(r[0],1)) + \
              '  ' + str(round(r[1],1)) + \
              '|  front'
        print '| ----- | | ----- |'
        print '|'  + str(round(l[3],1)) + \
              '  ' + str(round(l[2],1)) + \
              '| |'+ str(round(r[3],1)) + \
              '  ' + str(round(r[2],1)) + \
              '|  back'
        print '+-------+ +-------+'
        print 'total: %f Newtons, %f kilograms' \
              % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight)/9.81))

    def printFootBumpers(self):
        ll = self.lfootlbumper.getValue()
        lr = self.lfootrbumper.getValue()
        rl = self.rfootlbumper.getValue()
        rr = self.rfootrbumper.getValue()
        print '----------foot bumpers----------'
        print '+ left ------ right +'
        print '+--------+ +--------+'
        print '|'  + str(ll) + '  ' + str(lr) + '| |'+ str(rl) + '  ' + str(rr) + '|'
        print '|        | |        |'
        print '|        | |        |'
        print '+--------+ +--------+'

    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print '-----ultrasound sensors-----'
        print 'left: %f m, right %f m' % (dist[0], dist[1])

    def printCameraImage(self, camera):
        scaled = 2 # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()
        focal = camera.getFov()
        print "field of view ",
        print focal
        print '----------camera image (gray levels)---------'
        print 'original resolution: %d x %d, scaled to %d x %f' \
              % (width, height, width/scaled, height/scaled)

        for y in range(0, height/scaled):
            line = ''
            for x in range(0, width/scaled):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255 # between 0 and  instead of 0 and 255
                line = line + str(int(gray))
            print line

    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]

            if len(self.rphalanx) > i and self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

    def printHelp(self):
        print '----------nao_demo_python----------'
        print 'Use the keyboard to control the robots (one at a time)'
        print '(The 3D window need to be focused)'
        print '[Up][Down]: move one step forward/backwards'
        print '[<-][->]: side step left/right'
        print '[Shift] + [<-][->]: turn left/right'
        print '[U]: print ultrasound sensors'
        print '[A]: print accelerometers'
        print '[G]: print gyros'
        print '[S]: print gps'
        print '[I]: print inertial unit (roll/pitch/yaw)'
        print '[F]: print foot sensors'
        print '[B]: print foot bumpers'
        print '[Home][End]: print scaled top/bottom camera image'
        print '[PageUp][PageDown]: open/close hands'
        print '[7][8][9]: change all leds RGB color'
        print '[0]: turn all leds off'
        print '[H]: print this help message'

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getCamera("CameraTop")
        self.cameraBottom = self.getCamera("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getAccelerometer('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # gyro
        self.gyro = self.getGyro('gyro')
        self.gyro.enable(4 * self.timeStep)

        # gps
        self.gps = self.getGPS('gps')
        self.gps.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getInertialUnit('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ['Sonar/Left','Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDistanceSensor(usNames[i]))
            self.us[i].enable(self.timeStep)

        # foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getTouchSensor(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getTouchSensor('LFoot/Bumper/Left')
        self.lfootrbumper = self.getTouchSensor('LFoot/Bumper/Right')
        self.rfootlbumper = self.getTouchSensor('RFoot/Bumper/Left')
        self.rfootrbumper = self.getTouchSensor('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getLED('ChestBoard/Led'))
        self.leds.append(self.getLED('RFoot/Led'))
        self.leds.append(self.getLED('LFoot/Led'))
        self.leds.append(self.getLED('Face/Led/Right'))
        self.leds.append(self.getLED('Face/Led/Left'))
        self.leds.append(self.getLED('Ears/Led/Right'))
        self.leds.append(self.getLED('Ears/Led/Left'))

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getMotor("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getMotor("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        #head
        self.HeadYaw = self.getMotor("HeadYaw")
        self.HeadPitch = self.getMotor("HeadPitch")

        #leftarm
        self.LShoulderPitch = self.getMotor("LShoulderPitch");
        self.LShoulderRoll = self.getMotor("LShoulderRoll")
        self.LElbowYaw = self.getMotor("LElbowYaw")
        self.LElbowRoll = self.getMotor("LElbowRoll")

        #shouldarm
        self.RShoulderPitch = self.getMotor("RShoulderPitch");
        self.RShoulderRoll = self.getMotor("RShoulderRoll")
        self.RElbowYaw = self.getMotor("RElbowYaw")
        self.RElbowRoll = self.getMotor("RElbowRoll")

        #left leg
        self.LHipYawPitch = self.getMotor("LHipYawPitch")
        self.LHipRoll = self.getMotor("LHipRoll")
        self.LHipPitch = self.getMotor("LHipPitch")
        self.LKneePitch = self.getMotor("LKneePitch")
        self.LAnklePitch = self.getMotor("LAnklePitch")
        self.LAnkleRoll = self.getMotor("LAnkleRoll")

        #right leg
        self.RHipYawPitch = self.getMotor("RHipYawPitch")
        self.RHipRoll = self.getMotor("RHipRoll")
        self.RHipPitch = self.getMotor("RHipPitch")
        self.RKneePitch = self.getMotor("RKneePitch")
        self.RAnklePitch = self.getMotor("RAnklePitch")
        self.RAnkleRoll = self.getMotor("RAnkleRoll")

        self.RElbowRollS = self.getPositionSensor("RElbowRollS")
        self.RElbowRollS.enable(self.timeStep)
        self.LElbowRollS = self.getPositionSensor("LElbowRollS")
        self.LElbowRollS.enable(self.timeStep)
        self.HeadPitchS = self.getPositionSensor("HeadPitchS")
        self.HeadPitchS.enable(self.timeStep)


        self.RShoulderPitchS = self.getPositionSensor("RShoulderPitchS")
        self.RShoulderPitchS.enable(self.timeStep)
        self.RAnklePitchS = self.getPositionSensor("RAnklePitchS")
        self.RAnklePitchS.enable(self.timeStep)

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)
        #self.gps esta retornando nan
        #possivel fix: passar as coordenadas direto e nao o objeto gps como referencia

        self.odometryProvider = None
        # values = self.gps.getValues()
        # print("Values: %f %f" % (values[0], values[2]))
        # values = self.gps.getValues()
        # self.odometryProvider = BasicOdometryProvider(float(values[0]), float(values[2]))
        self.lastOdometry = OdometryData(0, 0, 0)
        # self.particleFilter = ParticleFilter()

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        self.shouldTurn120Degrees = False
        self.turnsMade = 0

        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.printHelp()


    def testSocketColor(self):
        # s.send("RECEBEESSAPORRA")
        # image = np.array(self.cameraTop.getImageArray())
        # dump = pickle.dumps(image)
        # compactLen = struct.pack("i", len(dump))
        # s.send(compactLen)
        # s.send(dump)
        stringImage = "GPS"
        dump = pickle.dumps(stringImage)
        compactLen = struct.pack("i", len(dump))
        s.send(compactLen)
        s.send(dump)
        p = self.gps.getValues()
        dump = pickle.dumps(p)
        compactLen = struct.pack("i", len(dump))
        s.send(compactLen)
        s.send(dump)
        # p = [self.lastOdometry.x, self.lastOdometry.z, self.lastOdometry[2])
        stringImage = "IMAGE"
        dump = pickle.dumps(stringImage)
        compactLen = struct.pack("i", len(dump))
        s.send(compactLen)
        s.send(dump)
        image = np.array(self.cameraTop.getImageArray())
        dump = pickle.dumps(image)
        compactLen = struct.pack("i", len(dump))
        s.send(compactLen)
        s.send(dump)

    def updateOdometry(self):
        # self.currentlyPlaying = None
        # print("updateOdometry")
        if (self.currentlyPlaying == self.turnLeft60):
            #send message
            self.lastOdometry = self.odometryProvider.rotationUpdate(False)
        elif (self.currentlyPlaying == self.turnRight60):
            self.lastOdometry = self.odometryProvider.rotationUpdate(True)
        else:
            values = self.gps.getValues()
            self.lastOdometry = self.odometryProvider.linearMovementUpdate(float(values[0]), float(values[2]))
        self.currentlyPlaying = False
        # print(self.lastOdometry)
        # stringImage = "ODOMETRY"
        # dump = pickle.dumps(stringImage)
        # compactLen = struct.pack("i", len(dump))
        # s.send(compactLen)
        # s.send(dump)
        # data = [self.lastOdometry.x, self.lastOdometry.z, self.lastOdometry.angle]
        # dump = pickle.dumps(data)
        # compactLen = struct.pack("i", len(dump))
        # s.send(compactLen)
        # s.send(dump)
        # self.particleFilter.updateParticlesWithOdometry(self.lastOdometry)

    def automaticMovement(self, currentX, currentZ):
        if (self.turnsMade == 1):
            if (self.currentlyPlaying is False):
                self.startMotion(self.turnRight60)
                self.turnsMade = 2
        elif (self.turnsMade == 2):
             if (self.currentlyPlaying is False):
                self.turnsMade = 0
                self.startMotion(self.forwards)
        elif (currentX + 0.6 > 4.5 or currentX - 0.6 < -4.5 or currentZ + 0.6 > 3.0 or currentZ - 0.6 < -3.0):
            #do two turns
            # self.currentlyPlaying.stop()
            self.startMotion(self.turnRight60)
            self.turnsMade = 1
        else:
            if (self.currentlyPlaying is False):
                self.startMotion(self.forwards)



    def run(self):
            shouldUseAutomaticMovement = True #turn to False if you want to control robot movement
            self.handWave.setLoop(True)
            self.handWave.play()
            # until a key is pressed
            key = -1

            while robot.step(self.timeStep) != -1:
                key = self.keyboard.getKey()
                if key > 0:
                    break
            self.handWave.setLoop(False)
            limitCounter = 0
            while True:
                # naive counter to avoid socket overload
                limitCounter = limitCounter + 1

                if self.odometryProvider is None:
                    values = self.gps.getValues()
                    # print(values)
                    if values[0] == values[0]:
                        print("Initializing odometry")
                        self.odometryProvider = BasicOdometryProvider(float(values[0]), float(values[2]))

                if (self.currentlyPlaying and self.currentlyPlaying.isOver()):
                    self.updateOdometry()

                if (shouldUseAutomaticMovement is True):
                    values = self.gps.getValues()
                    if values[0] == values[0]:
                        self.automaticMovement(float(values[0]), float(values[2]))

                key = self.keyboard.getKey()
                key = False
                if (limitCounter > 1):
                    self.testSocketColor()
                    limitCounter = 0;
                if key == Keyboard.LEFT:
                    self.startMotion(self.sideStepLeft)
                elif key == Keyboard.RIGHT:
                    self.startMotion(self.sideStepRight)
                elif key == Keyboard.UP:
                    self.startMotion(self.forwards)
                elif key == Keyboard.DOWN:
                    self.startMotion(self.backwards)
                elif key == Keyboard.LEFT | Keyboard.SHIFT:
                    self.startMotion(self.turnLeft60)
                elif key == Keyboard.RIGHT| Keyboard.SHIFT:
                    self.startMotion(self.turnRight60)
                elif key == ord('A'):
                    self.testSocket()
                elif key == ord('G'):
                    self.printGyro()
                elif key == ord('S'):
                    self.printGps()
                elif key == ord('I'):
                    self.printInertialUnit()
                elif key == ord('F'):
                    self.printFootSensors()
                elif key == ord('B'):
                    self.printFootBumpers()
                elif key == ord('U'):
                    self.printUltrasoundSensors()
                elif key == Keyboard.HOME:
                    self.printCameraImage(self.cameraTop)
                elif key == Keyboard.END:
                    self.printCameraImage(self.cameraBottom)
                elif key == Keyboard.PAGEUP:
                    self.setHandsAngle(0.96)
                elif key == Keyboard.PAGEDOWN:
                    self.setHandsAngle(0.0)
                elif key == ord('7'):
                    self.setAllLedsColor(0xff0000) # red
                elif key == ord('8'):
                    self.setAllLedsColor(0x00ff00) # green
                elif key == ord('9'):
                    self.setAllLedsColor(0x0000ff) # blue
                elif key == ord('0'):
                    self.setAllLedsColor(0x000000) # off
                elif key == ord('H'):
                    self.printHelp()

                if robot.step(self.timeStep) == -1:
                    break


# create socket to send data to server
HOST = 'localhost'
PORT = 50040
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
# create the Robot instance and run main loop
robot = Nao()
robot.run()
s.close()
