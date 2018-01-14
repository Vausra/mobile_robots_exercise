from math import *
import officeWorld
import Robot
import SensorUtilities
import numpy as np


# Roboter in Office-World positionieren:
myWorld = officeWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, [2, 5.5, pi/2])

# CursorController definieren:
myCursorController = myWorld.getCursorController()


# Bewege Roboter mit Cursor-Tasten:
while True:
    motion = myCursorController.getSpeed()
    if motion == None:
        break
    myRobot.move(motion)
    dists = myRobot.sense()
    directions = myRobot.getSensorDirections()
    lines_l = SensorUtilities.extractLinesFromSensorData(dists, directions)
    lines_g = SensorUtilities.transform(lines_l, myWorld.getTrueRobotPose())
    myWorld.drawPolylines(lines_g)

# Simulation schliessen:
myWorld.close(False)


