"""my_controller_supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor,Emitter,Node,Field,Keyboard
import math
import sys
import socket
import pickle
import struct

def getCorrectAngleFromRadianValue(radians):
  currentAngle = (math.degrees(radians) + 360) % 360 - 180
  if (currentAngle < 0):
    currentAngle = math.fabs(currentAngle)
  else:
    currentAngle = 360 - currentAngle
  return currentAngle
class Driver (Supervisor) :
  x = 0.1
  z = 0.3

  translationValues = [[0.75,0.32,-2],[0.75,0.32,-1.5],[0.75,0.32,-1.0],[0.75,0.32,-0.5],[0.75,0.32,0],
                      [0.75,0.32,0.5],[0.75,0.32,1.0],[0.75,0.32,1.5],[0.75,0.32,2]]

  secondTrans =  [[0.75,0.32,-2],[1.0,0.32,-2],[1.5,0.32,-2],[2,0.32,-2],[2.5,0.32,-2],[3,0.32,-2]]
  def wait(self):
      n = 0
      while (n < 200):
        n = n + 1
        if self.step(self.timeStep) == -1: break
  def initialization(self):
    self.timeStep = int(self.getBasicTimeStep())
    robot = self.getFromDef('PLAYER')
    # cameraTop = robot.getFromDef("CameraTop")
    # gps = robot.getFromDef('gps')
    # print gps.getValues()
    if robot is None:
      #robot might be None if the controller is about to quit
      sys.exit(1);


    # print robot.getField('translation').getSFVec3f()
    #robot.getField('rotation').setSFRotation([0.785094, 0.437966, 0.437965, 4.47276])
    #45 graus anti horario -0.281085 0.678598 0.678598 2.59356
    # robot.getField('rotation').setSFRotation([-0.862981, 0.357257, 0.357256, 1.71763])
    # robot.getField('rotation').setSFRotation([-1, 0, 0, 1.5708])#0 graus atan=0, atan2= -180
    robot.getField('rotation').setSFRotation([-0.57735, -0.57735, -0.57735, 2.0944]) #45graus, atan -45, atan2 135
    # robot.getField('rotation').setSFRotation([-0.57735, -0.57735, -0.57735, 2.0944]) #90graus  atan90, atan2 90
    # robot.getField('rotation').setSFRotation([-0.281085, -0.678598, -0.678598, 2.59356]) #135graus, atan 135, atan2 135
    # robot.getField('rotation').setSFRotation([-4.32978e-17, -0.707107, -0.707107, 3.14159]) #180 graus atan 0, atan2 = 0
    # robot.getField('rotation').setSFRotation([0.281085, -0.678598, -0.678598, 3.68962]) #225 graus atan -45, atan2 = -45
    # robot.getField('rotation').setSFRotation([0.57735, -0.57735, -0.57735, 4.18879]) #270 graus atan -90, atan2 = -90
    # robot.getField('rotation').setSFRotation([0.862856, -0.357407, -0.357407, 4.56541]) #315 graus atan 45, atan2 = -45
    # robot.getField('rotation').setSFRotation([-4.32978e-17, -0.707107, -0.707107, 3.14159]) #180 graus atan 0, atan2 = 0


    # self.wait()
    self.wait()
    #for value in self.translationValues:
     # robot.getField('translation').setSFVec3f(value)
      #self.wait()

    #45 graus horario
    #robot.getField('rotation').setSFRotation([-0.862981, -0.357257, -0.357256, 1.71763])

    # for value in self.secondTrans:
    #   robot.getField('translation').setSFVec3f(value)
    #   self.wait()
    currentX = float(robot.getField('translation').getSFVec3f()[0])
    currentZ = float(robot.getField('translation').getSFVec3f()[2])
    ori = robot.getOrientation()
    currentAngle = getCorrectAngleFromRadianValue(math.atan2(ori[1],ori[7]))
    # print "Angle: " + str(currentAngle)


    i = 0
    while(1):
      i += 1

      if (i%2 == 0):
        newAngle = getCorrectAngleFromRadianValue(math.atan2(ori[1],ori[7]))
        string = "ANGLE"
        dump = pickle.dumps(string)
        compactLen = struct.pack("i", len(dump))
        s.send(compactLen)
        s.send(dump)
        dump = pickle.dumps(newAngle)
        compactLen = struct.pack("i", len(dump))
        s.send(compactLen)
        s.send(dump)
      if (i%300 == 0):
        i = 0
        newX = float(robot.getField('translation').getSFVec3f()[0])
        newZ = float(robot.getField('translation').getSFVec3f()[2])
        ori = robot.getOrientation()
        newAngle = getCorrectAngleFromRadianValue(math.atan2(ori[1],ori[7]))
        # print "Angle: " + str(newAngle)
        odometryData = [newX-currentX, newZ-currentZ, newAngle-currentAngle]
        print odometryData
        stringImage = "ODOMETRY"
        dump = pickle.dumps(stringImage)
        compactLen = struct.pack("i", len(dump))
        s.send(compactLen)
        s.send(dump)
        dump = pickle.dumps(odometryData)
        compactLen = struct.pack("i", len(dump))
        s.send(compactLen)
        s.send(dump)
        currentX = newX
        currentZ = newZ
        currentAngle = newAngle
      # ori = robot.getOrientation()
      # angle = 180 - abs( math.degrees(math.atan2(ori[1],ori[7])))
      if self.step(self.timeStep) == -1: break

    #orientation = robot.getOrientation()
    #print orientation
    #orientation[1] = 0
    #orientation[7] = 0

    #print robot.getField('rotation').setSFRotation([-0.862981, 0.357257, 0.357256, 1.71763])
    #while(1):
     # ori = robot.getOrientation()
      #angle = 180 - abs( math.degrees(math.atan2(ori[1],ori[7])))
      #print angle
      #if self.step(self.timeStep) == -1: break
HOST = 'localhost'
PORT = 50040
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
controller = Driver()
controller.initialization()
s.close()
