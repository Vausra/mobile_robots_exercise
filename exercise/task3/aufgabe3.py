#!/usr/bin/python
import numpy as np

from HTWG_Robot_Simulator_AIN_V1 import Robot
from HTWG_Robot_Simulator_AIN_V1 import simpleWorld
from HTWG_Robot_Simulator_AIN_V1 import PlotUtilities
from HTWG_Robot_Simulator_AIN_V1 import OdometryPoseEstimator
from HTWG_Robot_Simulator_AIN_V1 import SensorUtilities
from ParticleFilterPoseEstimator import ParticleFilterPoseEstimator


myWorld = simpleWorld.buildWorld()

# this is the likelyhood field
myGrid = myWorld.getDistanceGrid()
myRobot = Robot.Robot()
arra = np.array(myGrid)
pose_estimator = ParticleFilterPoseEstimator.ParticleFilterPoseEstimator(myGrid)

#myGrid.drawGrid() # works, uncomment it to show the grid

def plot_result(pos_true, pos_odo):
    PlotUtilities.plotPositions(pos_true, 'k')
    PlotUtilities.plotPositions(pos_odo, 'r')
    PlotUtilities.plotShow()

if __name__ == '__main__':
    T = 0.1 # timestamp for robot
    pose_from = [8, 6, 0]
    pose_to = [10, 8, np.pi/2]

    print("distance grid generated")
    myRobot.setTimeStep(T)
    myWorld.setRobot(myRobot, [9, 7, 0])

    pose_estimator.initialize(pose_from, pose_to, 200)
    myWorld.drawPoints(pose_estimator.get_particles(), 'green')
    particles = pose_estimator.get_particles()

    dists = myRobot.sense()

    estimations = pose_estimator.calcCordsFromDistance(dists)

    for p in particles:
        for e in estimations:
            x_estimated = p[0] - e[0]
            y_estimated = p[1] - e[1]
            x_res = np.round(x_estimated)
            y_res = np.round(y_estimated)
            hood_value = myGrid.getValue(x_res, y_res)
            myWorld.drawPoint([x_estimated, y_estimated, 0], 'orange')

            if hood_value > 1:
                p[3] += 1


    best_particles = []
    for p in particles:
        if p[3] >= 10:
            best_particles.append(p)

    #myWorld.undrawPoints()

    myWorld.drawPoints(best_particles, 'red')

    myWorld.close(True)

    #n = 110

    #motionCircle = [[1, -24 * np.pi / 270] for i in range(n)]

    #for i in range(n):
        #motion = motionCircle[i]
        #myRobot.move(motion)
        #pose_estimator.integrate_movement(motion)

        #particles = pose_estimator.get_particles()
        #PlotUtilities.plotPositionParticles(particles)
