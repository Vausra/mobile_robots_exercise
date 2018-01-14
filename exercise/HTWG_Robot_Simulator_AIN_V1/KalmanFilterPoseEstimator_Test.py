# O. Bittel; 03.12.2017

from math import *
import numpy as np

import emptyWorld
import Robot
import PlotUtilities
import KalmanFilterPoseEstimator


# time step and number of time steps:
# T = 0.1 sec default value
n = 150


# Set a robot in a world and define landmarks:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
poseStart = [2,5,pi/2]
myWorld.setRobot(myRobot, poseStart)
lm = [[2,2], [8,2]]
myRobot.setLandmarks(lm)


# Movement definition:
motionCircle = [[1, -12*pi/180] for i in range(n)]

# n4 = int(n/4)
# motionCircleCurve = [[1, -12*pi/180] for i in range(n4)]
# motionCircleCurve.extend([[1, 12*pi/180] for i in range(n4)])
# motionCircleCurve.extend(motionCircleCurve)
#
# motion90RightTurn = [[1, -180*pi/180] for i in range(25)]
# motionSegment = [[1, 0] for i in range(n4 - 25)]
# motionSquare = []
# for i in range(4):
#     motionSquare.extend(motionSegment)
#     motionSquare.extend(motion90RightTurn)


# Define Kalman filter position estimator:
posEstimator = KalmanFilterPoseEstimator.KalmanFilterPoseEstimator()
poseStartEst = poseStart
poseStartEst[0] += 0.5
poseStartEst[1] += 0.5
poseStartEst[2] -= pi/18
posEstimator.setInitialPose(poseStartEst)
posEstimator.setInitialCovariance(np.array([[1**2, 0.0,    0.0],
                                            [0.0,    1**2, 0.0],
                                            [0.0,    0.0,    ((10/180.0)*pi)**2]]))
posEstimator.setLandmarks(lm)
#print(posEstimator.getPose())
#print(posEstimator.getCovariance())
#PlotUtilities.plotPoseCovariance(posEstimator.getPose(), posEstimator.getCovariance(), 'b')


# Move robot and estimate position:
pos_true = [myWorld.getTrueRobotPose()]         # True positions
pos_KalmanEstimated = [posEstimator.getPose()]  # Kalman Filter estimated positions

for t in range(n):
    # plot covariance:
    #if t % 10 == 0:
    #    PlotUtilities.plotPoseCovariance(posEstimator.getPose(), posEstimator.getCovariance(), 'k')

    # move real robot and sense:
    motion = motionCircle[t]
    myRobot.move(motion)
    sigma_motion = myRobot.getSigmaMotion()
    z = myRobot.senseLandmarks()
    sigma_z = myRobot.getSigmaSenseNoiseLandmarks()

    # add true pose
    pos_true.append(myWorld.getTrueRobotPose())

    # integrate movement and sensor measurement in the position estimation
    posEstimator.integrateMovement(motion,sigma_motion)
    #print(posEstimator.getPose())
    #print(sigma_z)
    #print(posEstimator.getCovariance())

    if t % 5 == 0:
        PlotUtilities.plotPoseCovariance(posEstimator.getPose(), posEstimator.getCovariance(), 'k')
        #print(posEstimator.getPose())
        #print(posEstimator.getCovariance())
        posEstimator.integrateMeasurement(z,sigma_z)
        #print(z)
        #print(sigma_z)
        PlotUtilities.plotPoseCovariance(posEstimator.getPose(), posEstimator.getCovariance(), 'g')
        #print(posEstimator.getPose())
        #print(posEstimator.getCovariance())

    # Add estimated pose:
    pos_KalmanEstimated.append(posEstimator.getPose())

# Simulation schliessen:
myWorld.close()

# plot true and estimated positions:
PlotUtilities.plotPositions(pos_true, 'k')
PlotUtilities.plotPositions(pos_KalmanEstimated, 'r')
PlotUtilities.plotPoseCovariance(posEstimator.getPose(), posEstimator.getCovariance(), 'r')
#print(posEstimator.getPose())
#print(posEstimator.getCovariance())
PlotUtilities.plotShow()

