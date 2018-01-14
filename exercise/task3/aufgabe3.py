#!/usr/bin/python
import numpy as np

from HTWG_Robot_Simulator_AIN_V1 import Robot
from HTWG_Robot_Simulator_AIN_V1.worlds import simpleWorld
from HTWG_Robot_Simulator_AIN_V1 import PlotUtilities
from HTWG_Robot_Simulator_AIN_V1 import OdometryPoseEstimator
from HTWG_Robot_Simulator_AIN_V1 import SensorUtilities
from ParticleFilterPoseEstimator import ParticleFilterPoseEstimator


myWorld = simpleWorld.buildWorld()

# this is the likelyhood field
myGrid = myWorld.getDistanceGrid()
myRobot = Robot.Robot()
pose_estimator = ParticleFilterPoseEstimator.ParticleFilterPoseEstimator(myGrid)



#myGrid.drawGrid() # works, uncomment it to show the grid

def plot_result(pos_true, pos_odo):
    PlotUtilities.plotPositions(pos_true, 'k')
    PlotUtilities.plotPositions(pos_odo, 'r')
    PlotUtilities.plotShow()

if __name__ == '__main__':
    T = 0.1 # timestamp for robot
    pose_from = 0
    pose_to = 20

    print("distance grid generated")
    myRobot.setTimeStep(T)
    myWorld.setRobot(myRobot, [4, 5.5, np.pi / 2])

    pose_estimator.initialize(pose_from, pose_to, 5)
    particles = pose_estimator.get_particles()

    PlotUtilities.plotPositionParticles(particles)
    PlotUtilities.plotShow()
    # Alle Raeume der Office world ausgeben:
    print(myWorld.getRooms())
    n = 110

    motionCircle = [[1, -24 * np.pi / 270] for i in range(n)]

    for i in range(n):
        motion = motionCircle[i]
        myRobot.move(motion)
        pose_estimator.integrate_movement(motion)

        particles = pose_estimator.get_particles()
        PlotUtilities.plotPositionParticles(particles)
