import numpy as np
import numpy.linalg as la
import random
import scipy.stats
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
        self._best_particles = [] # in every step a new list of best particles (chosen by max weight)

        self._covariance = 0

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
            p[3] = 1
            for index, dist in np.ndenumerate(dist_list):
                if dist is not None:
                    x_cord = dist * np.cos(alpha_list[index[0]] + p[2]) + p[0]
                    y_cord = dist * np.sin(alpha_list[index[0]] + p[2]) + p[1]
                    polar_coordinates.append([x_cord, y_cord, 0])
                    polar_coordinates.append([x_cord, y_cord, 0])

                    hood_value = distant_map.getValue(x_cord, y_cord)
                    if hood_value is None: # If measured point of a particle is negative
                        p[3] *= 0.00001
                        continue

                    probability = scipy.stats.norm(0, 0.5).pdf(hood_value)
                    p[3] *= probability

        self._polar_coordinates = polar_coordinates
        return polar_coordinates

    # calc average pose
    def get_pose(self):
        # TODO: Calc pose

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

    def resample(self):
        wheel = []
        weight_sum = 0
        randome_particles = []
        particles_old = self._particles

        for index in range(len(self._particles)):
            weight_sum += self._particles[index][3]
            wheel.append(weight_sum)

        # Roulett
        # TODO: implement binary search tree
        for _ in range(len(self._particles)):
            r = np.random.uniform(0, weight_sum)

            for index_weight in range(len(wheel)):
                if r <= wheel[index_weight]:
                    randome_particles.append(self._particles[index_weight])
                    break

        # TODO: Rework this. Set particles to new position with new direction
        for point_index in range(len(particles_old)):
            self._particles[0] = randome_particles[point_index][0]
            self._particles[1] = randome_particles[point_index][1]
            self._particles[2] = randome_particles[point_index][2]