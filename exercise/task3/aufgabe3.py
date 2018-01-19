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
arra = np.array(myGrid)
pose_estimator = ParticleFilterPoseEstimator.ParticleFilterPoseEstimator(myGrid)

#myGrid.drawGrid() # works, uncomment it to show the grid

def plot_result(pos_true, pos_odo):
    PlotUtilities.plotPositions(pos_true, 'k')
    PlotUtilities.plotPositions(pos_odo, 'r')
    PlotUtilities.plotShow()

if __name__ == '__main__':
    T = 0.1 # timestamp for robot

    r_orientation = np.pi / 2
    robot_initial_pose = [6, 6, r_orientation]


    pose_from = [robot_initial_pose[0]-1, robot_initial_pose[1]-1, robot_initial_pose[2] * 0]
    pose_to = [robot_initial_pose[0]+1, robot_initial_pose[1]+1, robot_initial_pose[2] * 1]

    print("distance grid generated")
    myRobot.setTimeStep(T)

    myWorld.setRobot(myRobot, robot_initial_pose)

    pose_estimator.initialize(pose_from, pose_to, 2)
    myWorld.drawPoints(pose_estimator.get_particles(), 'green')

    dists = myRobot.sense()
    sensor_directions = myRobot.getSensorDirections()

    polar_coordinates = pose_estimator.get_dist_list(dists, r_orientation)
    pose_estimator.set_weight_of_particle(myGrid)

    #pose_estimator.particles_match(dists, myGrid)

    weighted_particles = pose_estimator.get_particles()
    estimated_wall_hit_point = pose_estimator.get_estimated_wall_hit_point()

    #myWorld.drawPoints(estimated_wall_hit_point, 'orange') # draws a point on the coordinate where laser hits wall (seen by particle)

    n = 100

    motionCircle = [[1, -24 * np.pi / 270] for i in range(n)]

    for i in range(n):

        orientation = myWorld.getTrueRobotPose()[2]
        dists = myRobot.sense()
        polar_coordinates = pose_estimator.get_dist_list(dists, orientation)
        pose_estimator.set_weight_of_particle(myGrid)

        motion = motionCircle[i]
        myRobot.move(motion)
        pose_estimator.integrate_movement(motion)

        pose_estimator.analyze_particles()
        pose_estimator.reposition_particles()

        if i % 5 == 0:
            myWorld.undrawPoints()
            myWorld.drawPoints(pose_estimator.get_estimated_wall_hit_point(), 'orange')
            #myWorld.drawPoints(pose_estimator.get_particles(), 'brown')


    myWorld.close(True)
