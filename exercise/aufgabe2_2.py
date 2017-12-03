import numpy as np
import RoboLabor_AIN.HTWG_Robot_Simulator_AIN_V1.Robot as Robot
import RoboLabor_AIN.HTWG_Robot_Simulator_AIN_V1.emptyWorld as Empty_World
import math

# robot_position: actual position of the robot
# target_position: Target position for the robot
# radian: area around target position.
def check_target_area(robot_position, target, tolerance):
    return np.abs(target[0] - robot_position[0]) < tolerance and np.abs(target[1] - robot_position[1]) < tolerance


# returns gradient of the given vector
def gradient_of(vector):

    # to prevent division by zero
    if vector[1] == 0:
        return 0

    #if vector[0] == 0:
    #    return 1

    return vector[1] / vector[0]

#return a positive or negative faktor to get information about above or under line
def gradient_compare(vec_start_robot, vec_start_end):
    grad_1 = gradient_of(vec_start_robot)
    grad_2 = gradient_of(vec_start_end)

    if grad_1 < grad_2:
        return -1
    else: # grad_1 > grad_2:
        return 1


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
def calc_regulator(p_start, v_start_end, e_prev=0):

    p_robot = myWorld.getTrueRobotPose()

    v_start_robot = calc_vector_between_points(p_start, p_robot)

    alpha = angle_between_vector(v_start_robot, v_start_end)

    # | v_start_robot | -> we could say length of a vector
    v_start_robot_magnitude = vector_magnitude(v_start_robot)

    # determines if robot is above or under v_start_end
    # -1 means under, 1 means above, 0 means exact on the line -> maybe instead of return 0 return 1
    above_or_under = gradient_compare(v_start_robot, v_start_end)

    # result of this equation is the distance between robot and drawn polyline
    e = np.sin(alpha) * v_start_robot_magnitude * above_or_under

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
def follow_line(p_start, p_dest, vel,  tolerance, pd_regulator):

    v_start_end = [p_dest[0] - p_start[0], p_dest[1] - p_start[1]]

    robot_pos = myWorld.getTrueRobotPose()

    e_prev = 0

    while not check_target_area(robot_pos, p_dest, tolerance):

        p_regulator, d_regulator, e = calc_regulator(p_start, v_start_end, e_prev)

        if pd_regulator:
            omega = p_regulator - d_regulator
        else:
            omega = p_regulator

        myRobot.move([vel, omega])

        robot_pos = myWorld.getTrueRobotPose()

        e_prev = e

    print("Target Reached!")

# p_start: From where to start movement
# p_dest: Goal
# vel: Velocity
# tolerance: Tolerance for target area
# pd_regulator: with or without d-regulator
#
# go to a startpoint -> follow in imaginary line
def goto_global(p_start, p_dest, vel, tolerance, pd_regulator):

    #follow_line(_p_start, _p_end, TOLERANCE, pd_regulator=True)
    # check_target_area(robot_position, target, radian)

    robot_pos = myWorld.getTrueRobotPose()

    # Check if robot is very close to start position
    if not check_target_area(robot_pos, p_start, tolerance):

        tmp_start = robot_pos
        tmp_dest = p_start
        turn_robot_towards_target(tmp_dest)
        follow_line(tmp_start, tmp_dest, vel, tolerance, pd_regulator)

    turn_robot_towards_target(p_dest)
    follow_line(p_start, p_dest, vel, tolerance, pd_regulator)



# follow line after line
def follow_polyline(polyline, vel, tolerance, pd_regulator):

    for point in polyline:
        goto_global(point[0], point[1], vel, tolerance, pd_regulator)


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


if __name__ == "__main__":

    # Fahrthistorie -> world.py show path history
    # Setup world and place robot

    # Because T= 0.1 in the robot as default
    STEPS_PER_SEC = 5
    MAX_SPEED = 1
    SPEED = MAX_SPEED / STEPS_PER_SEC

    # Tolerance for target are
    TOLERANCE = 0.1

    # Weight for the two regulators
    # according to lecture 3 slide 32
    P_WEIGHT = -3  # p-regulator weight. NOTE: on the mentioned slide this weight is negative
    D_WEIGHT = 5 # d-regulator weight

    myWorld = Empty_World.buildWorld()
    myRobot = Robot.Robot()

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

    myWorld.drawPolyline([_p_start, _p_end_1, _p_end_2])

    # start with wrong direction
    # pi / 2 says rotation of the robot corresponding to the map (90 degrees)
    myWorld.setRobot(myRobot, [_p_start[0]+1, _p_start[1]+1, -2])

    #follow_line(_p_start, _p_end_1, SPEED, TOLERANCE, pd_regulator)
    #goto_global(_p_start, _p_end_1, SPEED, TOLERANCE, pd_regulator)
    follow_polyline(nav_points, SPEED, TOLERANCE, pd_regulator)

    # keep window open
    input()
