import sys

sys.path.insert(0, './RoboLabor_AIN/HTWG_Robot_Simulator_AIN_V1/')

import numpy as np
import RoboLabor_AIN.HTWG_Robot_Simulator_AIN_V1.Robot as Robot
import RoboLabor_AIN.HTWG_Robot_Simulator_AIN_V1.emptyWorld as Empty_World
import math
import time

# Because T= 0.1 in the robot as default
STEPS_PER_SEC = 10

myWorld = Empty_World.buildWorld()
myRobot = Robot.Robot()


# robot_position: actual position of the robot
# target_position: Target position for the robot
# radian: area around target position.
def check_target_area(robot_position, target, radian):
    return np.abs(target[0] - robot_position[0]) < radian and np.abs(target[1] - robot_position[1]) < radian


def curve_drive(velocity, radian, delta_theta):
    omega = velocity / radian
    steps = int(abs(delta_theta / omega) * abs(myRobot.getTimeStep() * 10))

    for i in range(0, steps):
        myRobot.move([velocity, omega])


# returns gradient of the given vector
def gradient_of(vector):
    if vector[1] == 0:
        return 0

    return vector[0] / vector[1]

#return a positive or negative faktor to get information about above or under line
def gradient_compare(vec_1, vec_2):
    grad_1 = gradient_of(vec_1)
    grad_2 = gradient_of(vec_2)

    if grad_1 < grad_2: # vectors are equal
        return -1
    elif grad_1 > grad_2:
        return 1
    else:
        return 0 # gradients are equal


# returns magnitude of a vector
def vector_length(vector):
    return np.sqrt(np.power(vector[0], 2) + np.power(vector[1], 2))


# returns angle between two vectors
# cos(alpha) = (vec_1 ° vec_2) / |vec_1| * |vec_2|
def angle_between_vector(v_1, v_2):

    scalar_product = np.dot(v_1, v_2)
    length_v1 = vector_length(v_1)
    length_v2 = vector_length(v_2)

    # compare float on equal is not possible
    if math.isclose(length_v2 * length_v2, 0.0, rel_tol=1e-5) \
            or math.isclose(scalar_product, 0.0, rel_tol=1e-5):
        return 0.0

    return np.arccos(scalar_product / (length_v1 * length_v2))


# p_start start point of the line to follow
# p_robot actual position of the robot
# p_end target position of the robot
# e_last which was calculated during previous measurement
#
# return distance between drawn line and robot
def regulator(p_start, p_end, p_robot, e_prev):
    v_start_robot = [p_robot[0] - p_start[0], p_robot[1] - p_start[1]]
    v_start_end = [p_end[0] - p_start[0], p_end[1] - p_start[1]]

    alpha = angle_between_vector(v_start_robot, v_start_end)

    # result of this equation is the distance between robot and drawn polyline
    e = np.sin(alpha) * vector_length(v_start_robot) * gradient_compare(v_start_robot, v_start_end)

    # compare float on equal is not possible
    if math.isclose(e, 0.0, rel_tol=1e-5) or math.isclose(alpha, 0.0, rel_tol=1e-5):
        return 0.0, 0.0

    p_regulator = -1 * e

    # Kd * ((e(t) - e(t - dt)) / dt)
    d_regulator = 50 * (e - e_prev)

    return p_regulator, d_regulator, e


# p1: start point
# p2: destination
def follow_line(p_start, p_destination):
    # this stays as it is. Angle between line and world
    end_point_theta = np.arctan2(p_destination[1] - p_start[1], p_destination[0] - p_start[0])

    # start angle
    robot_position = myWorld.getTrueRobotPose()
    e_prev = 0

    while not check_target_area(robot_position, p_destination, 0.3):
        robot_position = myWorld.getTrueRobotPose()

        p_regulator, d_regulator, e = regulator(p_start, p_destination, robot_position, e_prev)

        myRobot.move([0.1, p_regulator-d_regulator])
        e_prev = e

    print("Target Reached!")
    time.sleep(5)

# target point
#
def goto_global(v, p, tol):
    return 1


def follow_polyline(v, poly):
    return 1


if __name__ == "__main__":

    # TODO: Define target area -> matrix around the targetpoint 10x10


    # Fahrthistorie -> world.py show path history
    # Setup world and place robot


    start_point_x = 1.0
    start_point_y = 6.0

    end_point_x = 6.0
    end_point_y = 12.0

    _p_start = [start_point_x, start_point_y]
    _p_end = [end_point_x, end_point_y]

    myWorld.drawPolyline([_p_start, _p_end])

    # start with wrong direction
    # pi / 2 says rotation of the robot corresponding to the map (90 degrees)
    myWorld.setRobot(myRobot, [_p_start[0], _p_start[1], 15])

    follow_line(_p_start, _p_end)
