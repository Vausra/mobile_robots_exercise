from math import *
import officeWorld
import Robot
import World


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
    print("v = ", motion[0], "omega = ", motion[1]*180/pi)
    myRobot.move(motion)

# Simulation schliessen:
myWorld.close(False)