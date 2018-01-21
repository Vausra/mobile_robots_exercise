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
        self._T = 0.1  # time step
        self._particles = []

        self._polar_coordinates = []

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

            # Add noise to particle pose
            theta = self._particles[i][2]
            self._particles[i][0] += v * self._T * np.cos(theta)  # x
            self._particles[i][1] += v * self._T * np.sin(theta)  # y
            self._particles[i][2] = (self._particles[i][2] + omega * self._T) % (2 * np.pi)  # orientation

        return

    # weight all particles with likelihood-field and resample
    #
    # dist_list: Datas or robot sensors (distance to measured point, None if laser measured nothing)
    # alpha_list: Datas of robot sensors (angle to each other from right to left (negative to postive) as radians)
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
                    else:
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

    # TODO: wrong particle is picked
    def resample(self):
        wheel = [0]
        particles_old = self._particles

        for idx in range(len(self._particles)):
            wheel.append(wheel[idx] + self._particles[idx][3])

        # Roulett
        for idx in range(len(self._particles)):
            r = np.random.uniform(0, wheel[len(wheel) - 1])

            # Binary search
            # see https://en.wikipedia.org/wiki/Binary_search_algorithm
            left = 0
            right = len(particles_old) - 1
            while left <= right:
                m = int((left + right) / 2)

                if r < wheel[m]:
                    left = m + 1
                elif r > wheel[m]:
                    right = m - 1
                else:
                    break

            self._particles[idx][0] = particles_old[m][0]
            self._particles[idx][1] = particles_old[m][1]
            self._particles[idx][2] = particles_old[m][2]

            '''
            print(self._particles[idx])
            print("End")
            '''




