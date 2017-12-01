import sys
sys.path.insert(0, '../../RoboLabor_AIN/HTWG_Robot_Simulator_AIN_V1/')

import time
from math import *
import emptyWorld
import Robot

def curve_drive(velocity, radian, delta_theta):
    omega = velocity / radian

    # abs just to make sure that negative values not lead to an exception
    steps = int(abs(delta_theta / omega) * abs(myRobot.getTimeStep()*10))

    for i in range (0, steps):
        myRobot.move([velocity, omega])

# length should be a distance in meters
def straight_drive(velocity, length):

    distance = int(abs(myRobot.getTimeStep()*10)) * length * 10

    for i in range(distance):
       myRobot.move([velocity, 0])


def run_randome(velocity, radian, delta_theta):
    return 1

if __name__ == "__main__":

    # Fahrthistorie -> world.py show path history
    # Setup world and place robot
    myWorld = emptyWorld.buildWorld()
    myRobot = Robot.Robot()
    myWorld.setRobot(myRobot, [2, 5.5, pi / 2]) # pi / 2 says rotation of the robot corresponding to the map (90 degrees)

    # number of movements we want to do
    NUMBER_MOVEMENTS = 7
    # how many steps we want to do. But this is calculated in distance as meters
    n = 2

    # motion_circle = [[1, -24 * pi / 180]] #24 degree per step
    # T = 0.1
    # omega = 25 Grad/sec
    # n = 150
    # -> 360 Grad
    '''
    straigth_drive(0.1, n)
    curve_drive(0.1, 2, pi)
    straigth_drive(0.1, n)
    curve_drive(0.1, -2, pi)
    straigth_drive(0.1, n)
    '''

    for movement in range(0, NUMBER_MOVEMENTS):
        mv = movement % 3
        if  mv == 0:
            straight_drive(0.1, n)
        elif mv == 1:
            curve_drive(0.1, 2, pi)
        else:
            curve_drive(0.1, -2, pi)

    print("Movement done")
