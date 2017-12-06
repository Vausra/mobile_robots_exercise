import numpy as np
import math
import sys
sys.path.append('./RoboLabor_AIN/HTWG_Robot_Simulator_AIN_V1')

import Robot as Robot
import emptyWorld as Empty_World
import OdometryPoseEstimator as Odometry_Pose_Estimator
import SensorUtilities as Sensor_Utilities

# robot_position: actual position of the robot
# target_position: Target position for the robot
# tolerance: area around target position.
def check_target_area(robot_position, target, tolerance):
    return np.abs(target[0] - robot_position[0]) < tolerance and np.abs(target[1] - robot_position[1]) < tolerance


#also determines left or right
def above_or_under(p_start, p_end, p_robot):
    d_x = p_end[0] - p_start[0]
    d_y = p_start[1] - p_end[1]

    p_x = p_robot[0]
    p_y = p_robot[1]

    res = (p_x - p_start[0]) * d_y + (p_y - p_start[1]) * d_x

    if res == 0:
        return 0
    elif res < 0:
        return -1 # under the line
    else:
        return 1 # above line


# returns a vector between two points
def calc_vector_between_points(point_1, point_2):
    return [point_2[0] - point_1[0], point_2[1] - point_1[1]]


# returns magnitude ( |vector| ) of a vector
def vector_magnitude(vector):
    return np.sqrt(np.power(vector[0], 2) + np.power(vector[1], 2))


# returns angle between two vectors
# cos(alpha) = (vec_1 ° vec_2) / |vec_1| * |vec_2|
def angle_between_vector(v_1, v_2):

    scalar_product = np.dot(v_1, v_2) # v_1[0] * v_2[0] + v_1[1] * v_2[1]
    length_v1 = vector_magnitude(v_1)
    length_v2 = vector_magnitude(v_2)

    product = length_v1 * length_v2

    # compare float on equal is not possible
    if math.isclose(product, 0.0, rel_tol=1e-5) \
            or math.isclose(scalar_product, 0.0, rel_tol=1e-5):
        return 0.0

    quotient = scalar_product / product

    return np.arccos(quotient)


# p_start start point of the line to follow
# p_end target position of the robot
# e_last which was calculated during previous measurement
#
# This method uses the actual robot position.
#
# return distance between drawn line and robot
def calc_regulator(p_start, p_end, v_start_end, e_prev=0):

    p_robot = myWorld.getTrueRobotPose()

    v_start_robot = calc_vector_between_points(p_start, p_robot)

    alpha = angle_between_vector(v_start_robot, v_start_end)

    # | v_start_robot | -> we could say length of a vector
    v_start_robot_magnitude = vector_magnitude(v_start_robot)

    # determines if robot is above or under v_start_end
    # -1 means under, 1 means above, 0 means exact on the line
    a_or_u = above_or_under(p_start, p_end, p_robot)

    # result of this equation is the distance between robot and drawn polyline
    e = np.sin(alpha) * v_start_robot_magnitude * a_or_u

    # compare float on equal is not possible
    if math.isclose(e, 0.0, rel_tol=1e-5) or math.isclose(alpha, 0.0, rel_tol=1e-5):
        return 0.0, 0.0, 0.0

    p_regulator = P_WEIGHT * e
    d_regulator = D_WEIGHT * (e - e_prev)

    return p_regulator, d_regulator, e


# p_start: start point
# p_dest: destination
# vel: Velocity
# tolerance: How far robot can be away from its "p_dest"
# pd_regulator: with or without d-regulator
def follow_line(p_start, p_end, tolerance, pd_regulator):

    v_start_end = [p_end[0] - p_start[0], p_end[1] - p_start[1]]

    robot_pos = myWorld.getTrueRobotPose()

    e_prev = 0

    while not check_target_area(robot_pos, p_end, tolerance):

        vel = np.sqrt(np.power(robot_pos[0] - p_end[0], 2) + np.power(robot_pos[1] - p_end[1], 2))

        p_regulator, d_regulator, e = calc_regulator(p_start, p_end, v_start_end, e_prev)

        if pd_regulator:
            omega = p_regulator - d_regulator
        else:
            omega = p_regulator

        move_command = [vel, omega]

        myRobot.move(move_command)

        pos_estimator.integrateMovement(move_command, myRobot.getSigmaMotion())

        robot_pos = myWorld.getTrueRobotPose()

        e_prev = e

    print("Target Reached!")

# if the robot doesn't point to target turn it as long as it does
def turn_robot_towards_target(p_dest):
    p_robot = myWorld.getTrueRobotPose()
    angle_world_point = np.abs(np.arctan2(p_dest[1] - p_robot[1], p_dest[0] - p_robot[0]) * 180 / math.pi)


    while True:
        robot_angle_world = np.abs(p_robot[2] * 180 / math.pi)

        if math.isclose(robot_angle_world, angle_world_point, rel_tol=1e-1):
            break
        myRobot.move([0, 1])
        p_robot = myWorld.getTrueRobotPose()


def follow_wall(p_dest, tolerance):
    while not check_target_area(pos_estimator.getPose(), p_dest, tolerance):
        walls = detect_walls()
        left_wall_rel = get_left_wall(walls)

        if left_wall_rel is None:
            break

        left_wall_pres = Sensor_Utilities.transform([left_wall_rel], myWorld.getTrueRobotPose())[0]
        myWorld.drawPolylines([left_wall_pres])

        left_wall = [[round(left_wall_pres[0][0], 1), round(left_wall_pres[0][1], 1)],
                     [round(left_wall_pres[1][0], 1), round(left_wall_pres[1][1], 1)]]

        target_line = parallel_line(left_wall, 0.5)
        follow_line(target_line[1], target_line[0], SPEED, target_line[0], TOLERANCE)


def detect_walls():
    dists = myRobot.sense()
    directions = myRobot.getSensorDirections()
    return Sensor_Utilities.extractLinesFromSensorData(dists, directions)


# Berechnet den absoluten Winkel einer Line
def calc_line_angle(line):
    p1 = line[0]
    p2 = line[1]

    delta_x = p2[0] - p1[0]
    delta_y = p2[1] - p1[1]

    if delta_x == 0:
        return  # TODO

    return math.atan(delta_y / delta_x)


# Gibt eine parallele Linie mit Abstand dist zurueck.
def parallel_line(line, dist):
    alpha = calc_line_angle(line)

    alpha = abs(alpha)

    if alpha == 0:
        dist_x = 0
        dist_y = dist

    elif math.degrees(alpha) == 90:
        dist_x = dist
        dist_y = 0

    elif math.degrees(alpha) < 45:
        dist_x = dist / math.sin(alpha)
        dist_y = dist / math.cos(alpha)

    elif math.degrees(alpha) > 45:
        beta = 180 - 90 - alpha
        dist_x = math.cos(beta) * dist
        dist_y = dist / math.sin(beta)

    elif math.degrees(alpha) == 45:
        dist_x = dist / math.sin(alpha)
        dist_y = dist_x

    return [[line[0][0] - dist_x, line[0][1] - dist_y], [line[1][0] - dist_x, line[1][1] - dist_y]]


# Gets a List of walls and returns the left wall
def get_left_wall(walls):
    for wall in walls:
        p2x_wall = wall[0][0]
        p2y_wall = wall[0][1]
        if p2y_wall > 0 < p2x_wall:  # Ist eine Linke Wand, da y < 0
            return wall


if __name__ == "__main__":

    # Fahrthistorie -> world.py show path history
    # Setup world and place robot

    # Because T= 0.1 in the robot as default
    STEPS_PER_SEC = 5
    MAX_SPEED = 1
    SPEED = MAX_SPEED / STEPS_PER_SEC

    # Tolerance for target are
    TOLERANCE = 0.2

    # Weight for the two regulators
    # according to lecture 3 slide 32
    P_WEIGHT = -3  # p-regulator weight. NOTE: on the mentioned slide this weight is negative
    D_WEIGHT = 5 # d-regulator weight

    myWorld = Empty_World.buildWorld()
    myRobot = Robot.Robot()

    pos_estimator = Odometry_Pose_Estimator.OdometryPoseEstimator()
    sigma_pose = np.zeros((3, 3))
    sigma_pose[0, 0] = 0.2 ** 2
    sigma_pose[1, 1] = 0.2 ** 2
    sigma_pose[2, 2] = (5 * math.pi / 180) ** 2


    # set this false to run without d-regulator
    pd_regulator = True

    start_point_x = 1.0
    start_point_y = 6.0

    end_point_1_x = 6.0
    end_point_1_y = 12.0

    end_point_2_x = 10.0
    end_point_2_y = 15.0

    _p_start = [start_point_x, start_point_y]
    _p_end_1 = [end_point_1_x, end_point_1_y]
    _p_end_2 = [end_point_2_x, end_point_2_y]

    nav_points = [[_p_start, _p_end_1],[_p_end_1, _p_end_2]]

    ROBOT_START_POSE = [_p_start[0], _p_start[1]-1, math.pi]

    myWorld.drawPolyline([_p_start, _p_end_1, _p_end_2])

    pos_estimator.setInitialCovariance(sigma_pose)
    pos_estimator.setInitialPose(ROBOT_START_POSE)

    # start with wrong direction
    # pi / 2 says rotation of the robot corresponding to the map (90 degrees)
    myWorld.setRobot(myRobot, ROBOT_START_POSE)

    follow_line(_p_start, _p_end_1, TOLERANCE, pd_regulator)
    #follow_wall([10.5, 6], TOLERANCE)

    # keep window open
    input()
