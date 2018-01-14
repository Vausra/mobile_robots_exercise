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

    # ---------------Hesse form----------------------#
    #
    # alpha = np.arctan2(p_end[0] - p_start[0], p_end[1] - p_start[1])
    # radius = p_start[0] * np.sin(alpha) + p_start[1] * np.cos(alpha)
    # dist = p_robot[0] * np.sin(alpha) + p_robot[1] * np.cos(alpha) - radius
    #
    # ---------------End Hesse form----------------------#

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
def follow_line(p_start, p_end, tolerance, p_global_dest):

    v_start_end = [p_end[0] - p_start[0], p_end[1] - p_start[1]]

    robot_pos = myWorld.getTrueRobotPose()

    e_prev = 0

    while not check_target_area(robot_pos, p_global_dest, tolerance):

        vel = np.sqrt(np.power(robot_pos[0] - p_end[0], 2) + np.power(robot_pos[1] - p_end[1], 2))

        p_regulator, d_regulator, e = calc_regulator(p_start, p_end, v_start_end, e_prev)

        omega = p_regulator - d_regulator

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


# Aufgabe 3 Wandverfolgung
# Gibt alle erkannten Waende als Liste zurueck
# [p1_wall1, p2_wall1]
# [[x1_wall1, y1_wall1], [x2_wall1, y2_wall1]]
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

    if delta_x == 0:  # Linie ist senkrecht
        return None

    return math.atan(delta_y / delta_x)


# Gibt eine parallele Linie mit Abstand dist zurueck.
def parallel_line(line, dist):
    alpha = calc_line_angle(line)

    if alpha is not None:
        alpha = abs(alpha)

    if alpha is None:  # Linie ist senkrecht
        dist_x = dist
        dist_y = 0

    elif alpha == 0:  # Linie ist waagrecht
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

    p1x = line[0][0]
    p1y = line[0][1]

    p2x = line[1][0]
    p2y = line[1][1]

    # X-Richtung des Roboters
    if p1x < p2x:  # -x
        p1y = p1y + dist_y
        p2y = p2y + dist_y
    else:  # +x
        p1y = p1y - dist_y
        p2y = p2y - dist_y

    # Y-Richtung des Roboters
    if p1y > p2y:  # +y
        p1x = p1x + dist_x
        p2x = p2x + dist_x
    else:  # -y
        p1x = p1x - dist_x
        p2x = p2x - dist_x

    p1 = [p1x, p1y]
    p2 = [p2x, p2y]

    return [p1, p2]


# Gibt die front wall aus einer Liste von Waenden zurueck
def get_front_wall(walls):
    for wall in walls:
        p1x_wall = wall[1][0]
        p1y_wall = wall[1][1]
        p2x_wall = wall[0][0]
        p2y_wall = wall[0][1]
        if p1y_wall > 0 < p1x_wall and p2y_wall < 0 < p2x_wall:
            return wall


# Gibt die left wall aus einer Liste von Waenden zurueck
# Falls front wall naeher als WALL_DIST return front wall
def get_wall(walls, left_or_right, front_dist):
    front_wall = get_front_wall(walls)
    if front_wall is not None and front_wall[0][0] < front_dist:
        return front_wall

    for wall in walls:
        p2x_wall = wall[0][0]
        p2y_wall = wall[0][1]
        if left_or_right == "left":
            if p2y_wall > 0 < p2x_wall:  # left wall: y < 0
                return wall
        else:
            if p2y_wall < 0 > p2x_wall:  # right wall: y > 0
                return wall


def follow_wall(left_or_right, tolerance, wall_dist, front_dist, p_global_dest):
    while True:
        walls = detect_walls()
        wall_relative = get_wall(walls, left_or_right, front_dist)

        if wall_relative is None:
            break

        wall_abs = Sensor_Utilities.transform([wall_relative], myWorld.getTrueRobotPose())[0]

        wall = [[wall_abs[0][0], wall_abs[0][1]],
                [wall_abs[1][0], wall_abs[1][1]]]

        target_line = parallel_line(wall, wall_dist)
        myWorld.drawPolylines([wall_abs, target_line])

        if left_or_right is 'left':
            follow_line(target_line[1], target_line[0], [target_line[0]], tolerance)
        else:
            follow_line(target_line[0], target_line[1], [target_line[1]], tolerance)




def draw_world():

    # outer lines
    myWorld.addLine(19, 1, 19, 19)
    myWorld.addLine(1, 1, 19, 1)
    myWorld.addLine(19, 19, 1, 19)
    myWorld.addLine(1, 1, 1, 19)

    # inner lines
    myWorld.addLine(3, 3, 17, 3)
    myWorld.addLine(17, 3, 17, 17)
    myWorld.addLine(17, 17, 3, 17)
    myWorld.addLine(3, 3, 3, 17)


if __name__ == "__main__":

    # Fahrthistorie -> world.py show path history
    # Setup world and place robot

    # Because T= 0.1 in the robot as default
    STEPS_PER_SEC = 5

    # Tolerance for target area
    TOLERANCE = 0.2

    # Weight for the two regulators
    # according to lecture 3 slide 32
    P_WEIGHT = -0.2  # p-regulator weight. NOTE: on the mentioned slide this weight is negative
    D_WEIGHT = 15 # d-regulator weight

    # which wall to follow: "right" or "left"
    left_or_right = "right"

    wall_dist = 1
    front_dist = 1

    p_global_dest = [2.0, 4.0]

    myWorld = Empty_World.buildWorld()
    myRobot = Robot.Robot()

    pos_estimator = Odometry_Pose_Estimator.OdometryPoseEstimator()
    sigma_pose = np.zeros((3, 3))
    sigma_pose[0, 0] = 0.2 ** 2
    sigma_pose[1, 1] = 0.2 ** 2
    sigma_pose[2, 2] = (5 * math.pi / 180) ** 2

    start_point_x = 4.0
    start_point_y = 2.0

    ROBOT_START_POSE = [start_point_x, start_point_y, math.pi/180]

    pos_estimator.setInitialCovariance(sigma_pose)
    pos_estimator.setInitialPose(ROBOT_START_POSE)

    draw_world()

    myWorld.setRobot(myRobot, ROBOT_START_POSE)

    #follow_wall(left_or_right, TOLERANCE, wall_dist, front_dist, p_global_dest)


    # keep window open
    input()
