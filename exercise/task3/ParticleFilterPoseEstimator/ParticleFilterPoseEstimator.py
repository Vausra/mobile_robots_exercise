import numpy as np
import numpy.linalg as la
import random
import Particle
from HTWG_Robot_Simulator_AIN_V1 import World


class ParticleFilterPoseEstimator:

    # noise says how strong the random noise for each particle should be
    def __init__(self):

        # protected variables
        self._pose = []
        self._sigma_motion = np.zeros((3, 3))
        self._T = 0.1  # time step
        self._k_theta = 0.01
        self._particles = []
        self._covariance = 0
        self._polar_coordinates = []
        self._estimated_wall_hit_point = []
        self._max_weight_point = []

        self._sigma_noise = np.zeros((2, 2))

        # TODO: Set this by param
        self._sigma_noise[0, 0] = 0.01  # 0.2 ** 2
        self._sigma_noise[1, 1] = 0.01  # 0.2 ** 2
        # self._sigma_noise[2, 2] = (5 * np.pi / 180) ** 2

    # create random particles in area (pose_from, pose_to)
    def initialize(self, pose_from, pose_to, n=200):

        for i in range(n):

            position_x = random.uniform(pose_from[0], pose_to[0])
            position_y = random.uniform(pose_from[1], pose_to[1])
            orientation = random.uniform(pose_from[2], pose_to[2])

            self._particles.append([position_x, position_y, orientation, 0])

        return self._particles

    # return the list of particles
    def get_particles(self):
        return self._particles

    # returns the point where laser sensor hits the wall
    def get_estimated_wall_hit_point(self):
        return self._estimated_wall_hit_point

    # run motion command on all particles
    # Reminder: motion -> [velocity, omega]
    def integrate_movement(self, motion):
        for i in range(len(self._particles)):
            v = random.gauss(motion[0], np.sqrt(self._sigma_noise[0, 0]))
            omega = random.gauss(motion[1], np.sqrt(self._sigma_noise[1, 1]))
            # Apply noisy motion to particle _particles[i]:
            theta = self._particles[i][2]
            self._particles[i][0] += v * self._T * np.cos(theta)  # x
            self._particles[i][1] += v * self._T * np.sin(theta)  # y
            self._particles[i][2] = (self._particles[i][2] + omega * self._T) % (2 * np.pi)  # orientation

        return

    # weight all particles with likelihoodfield and resample
    #
    # dist_list: Datas or robot sensors (distance to measured point, None if laser measured nothing)
    # alpha_list: Datas of robot sensors (angle ??)
    # distant_map: Type of 'myWorld.getDistanceGrid()'
    def integrated_measurement(self, dist_list, alpha_list, distant_map):

        polar_coordinates = []
        for p in self._particles: # iterate over pixels and transform robot laser beams
            for index, dist in np.ndenumerate(dist_list):
                if dist is not None:
                    x_cord = dist * np.cos(alpha_list[index[0]] + p[2]) + p[0]
                    y_cord = dist * np.sin(alpha_list[index[0]] + p[2]) + p[1]
                    polar_coordinates.append([x_cord, y_cord, 0])
                    polar_coordinates.append([x_cord, y_cord, 0])

                    hood_value = distant_map.getValue(x_cord, y_cord)
                    if hood_value is None: # If measured point of a particle is negative
                        continue

                    # set the actual weight of a particle
                    if hood_value < 1:
                        p[3] += 1

        self._polar_coordinates = polar_coordinates
        return polar_coordinates

    # calc average pose
    def get_pose(self):
        return self._pose

    # calc covarianz inside particle set
    # have a look at "Aufgabe_4_3.py line 43 - 54
    def get_covariance(self):
        x = np.array(range(len(self._particles)))
        y = np.array(range(len(self._particles)))
        i = 0
        for p in self._particles:
            x[i] = p[0]
            y[i] = p[1]
            i += 1
        cov = np.array(np.cov([x, y]))

        self._covariance = cov

        return cov

    # Takes robot sensor data transfer it to polarcoordinates of particle
    #
    # return polar_coordinate as array where laser hits wall (seen by robot sensors)
    def get_dist_list(self, senseData, robot_orientation):
        polar_coordinates = []
        for index, s in np.ndenumerate(senseData):
            if s is not None:
                grad = (index[0] * 10) + 45 + np.degrees(robot_orientation)
                x_cord = s * np.cos(np.radians(grad))
                y_cord = s * np.sin(np.radians(grad))
                polar_coordinates.append([x_cord, y_cord, 0])
        self._polar_coordinates = polar_coordinates
        return polar_coordinates


    def set_weight_of_particle(self, grid):    # instead: set_weight method ??
        for p in self._particles:
            for e in self._polar_coordinates: # wall hit pint particle polar coordinate + dist of a laser
                x_estimated = p[0] - e[0]
                y_estimated = p[1] - e[1]

                self._estimated_wall_hit_point.append([x_estimated, y_estimated, 0])

                x_res = np.round(x_estimated)
                y_res = np.round(y_estimated)
                hood_value = grid.getValue(x_res, y_res)

                # set the actual weight of a particle
                if hood_value < 1:
                    p[3] += 1


    def particles_match(self, senseData, grid):
        for p in self._particles:
            for index, s in np.ndenumerate(senseData):
                if s is not None:
                    grad = (index[0] * 10) + 45 + np.degrees(p[3])
                    x_cord = s * np.cos(np.radians(grad))
                    y_cord = s * np.sin(np.radians(grad))

                    x_estimated = p[0] - x_cord
                    y_estimated = p[1] - y_cord

                    self._estimated_wall_hit_point.append([x_estimated, y_estimated, 0])

                    x_res = np.round(x_estimated)
                    y_res = np.round(y_estimated)
                    hood_value = grid.getValue(x_res, y_res)

                    if hood_value >= 1 and hood_value < 2:
                        p[3] += 1


    # get the particle with the maximum weight (just pick one if several have the same weight)
    def analyze_particles(self):
        self._max_weight_point = [0,0,0,0]

        for p in self._particles:
            if p[3] >= self._max_weight_point[3]:
                self._max_weight_point = p


    def reposition_particles(self):
        for p in self._particles:
            p[0] = self._max_weight_point[0]
            p[1] = self._max_weight_point[1]
            p[2] = self._max_weight_point[2]