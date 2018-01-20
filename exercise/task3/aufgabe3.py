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
distance_map = myWorld.getDistanceGrid()

myRobot = Robot.Robot()
pose_estimator = ParticleFilterPoseEstimator.ParticleFilterPoseEstimator()

#myGrid.drawGrid() # works, uncomment it to show the grid

def plot_result(pos_true, pos_odo):
    PlotUtilities.plotPositions(pos_true, 'k')
    PlotUtilities.plotPositions(pos_odo, 'r')
    PlotUtilities.plotShow()

if __name__ == '__main__':
    T = 0.1 # timestamp for robot

    distance_map.printGrid()

    r_orientation = np.pi / 2
    robot_initial_pose = [7, 7, r_orientation]


    pose_from = [robot_initial_pose[0], robot_initial_pose[1], robot_initial_pose[2] * 1]
    pose_to = [robot_initial_pose[0], robot_initial_pose[1], robot_initial_pose[2] * 1]

    print("distance grid generated")
    myRobot.setTimeStep(T)

    myWorld.setRobot(myRobot, robot_initial_pose)

    pose_estimator.initialize(pose_from, pose_to, 1)
    myWorld.drawPoints(pose_estimator.get_particles(), 'green')

    dist_list = myRobot.sense()
    alpha_list = myRobot.getSensorDirections()

    test = pose_estimator.integrated_measurement(dist_list, alpha_list, distance_map)
    myWorld.drawPoints(test, 'orange')

    n = 100

    motionCircle = [[1, -24 * np.pi / 270] for i in range(n)]

    for i in range(n):

        dist_list = myRobot.sense()
        alpha_list = myRobot.getSensorDirections()

        motion = motionCircle[i]
        myRobot.move(motion)
        pose_estimator.integrate_movement(motion)

        if i % 5 == 0:
            myWorld.undrawPoints()
            test = pose_estimator.integrated_measurement(dist_list, alpha_list, distance_map)
            myWorld.drawPoints(test, 'orange') # draw hitpoints: laser -> wall

            pose_estimator.resample()
            myWorld.drawPoints(pose_estimator.get_particles(), 'brown') # Draw resampled particles NOTE: For now does not work


    myWorld.close(True)
