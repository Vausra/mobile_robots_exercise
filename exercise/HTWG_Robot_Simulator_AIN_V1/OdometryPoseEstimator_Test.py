from math import *
import numpy as np

import emptyWorld
import Robot
import PlotUtilities
import OdometryPoseEstimator

# time step and number of time steps:
# T = 0.1 sec default value
n = 141

# Set a robot in a world
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
poseStart = [2,5,pi/2]
myWorld.setRobot(myRobot, poseStart)


# Movement definition:
motionCircle = [[1, -24 * pi / 180] for i in range(n)]

n4 = int(n/4)
motionCircleCurve = [[1, -12*pi/180] for i in range(n4)]
motionCircleCurve.extend([[1, 12*pi/180] for i in range(n4)])
motionCircleCurve.extend(motionCircleCurve)

motion90RightTurn = [[1, -180*pi/180] for i in range(25)]
motionSegment = [[1, 0] for i in range(n4 - 25)]
motionSquare = []
for i in range(4):
    motionSquare.extend(motionSegment)
    motionSquare.extend(motion90RightTurn)


# Define Odometry position estimator:
posEstimator = OdometryPoseEstimator.OdometryPoseEstimator()
sigma_pose = np.zeros((3,3))
sigma_pose[0,0] = 0.2**2
sigma_pose[1,1] = 0.2**2
sigma_pose[2,2] = (5*pi/180)**2
posEstimator.setInitialCovariance(sigma_pose)
posEstimator.setInitialPose(poseStart)
#posEstimator.setTimestep(T)



# Move robot and plot true and odo poses:
pos_odo = [myWorld.getTrueRobotPose()]
pos_true = [posEstimator.getPose()]

for t in range(n):
    # plot covariance:
    if t % 10 == 0:
        PlotUtilities.plotPoseCovariance(posEstimator.getPose(), posEstimator.getCovariance(), 'b')

    # move robot
    motion = motionCircle[t]
    print("v = ", motion[0], "omega = ", motion[1]*180/pi)
    myRobot.move(motion)

    # add true pose
    pos_true.append(myWorld.getTrueRobotPose())

    # odo pose:
    posEstimator.integrateMovement(motion,myRobot.getSigmaMotion())
    pos_odo.append(posEstimator.getPose())

    # Gib Daten vom Distanzsensor aus:
    distanceSensorData = myRobot.sense()
    print("Dist Sensor: ", distanceSensorData)

# Simulation schliessen:
myWorld.close()

# Plot poses:
PlotUtilities.plotPositions(pos_true, 'k')
PlotUtilities.plotPositions(pos_odo, 'r')
PlotUtilities.plotShow()

