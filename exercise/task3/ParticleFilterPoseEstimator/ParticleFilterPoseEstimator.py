import numpy as np
import random
import scipy.stats
from bisect import bisect_left

class ParticleFilterPoseEstimator:

    # noise says how strong the random noise for each particle should be
    def __init__(self, world, sigma_motion, time_step):

        # protected variables
        self._pose = []
        self._T = time_step
        self._particles = []
        self._number_of_particles = 0
        self._polar_coordinates = []
        self._covariance = 0
        self.sigma_motion = sigma_motion
        self.average = [0.0, 0.0, 0.0]
        self.world = world
        self.covariance = np.zeros((3, 3))
        return

    # create random particles in area (pose_from, pose_to)
    def initialize(self, pose_from, pose_to, n=200):
        #self._particles = [] # just in case robot lost his position
        self._number_of_particles = n
        for i in range(n):

            position_x = random.random() * (pose_to[0] - pose_from[0]) + pose_from[0]
            position_y = random.random() * (pose_to[1] - pose_from[1]) + pose_from[1]
            orientation = random.random() * (pose_to[2] - pose_from[2]) + pose_from[2]

            self._particles.append([position_x, position_y, orientation, 0])

        return


    # return the list of particles
    def get_particles(self):
        return self._particles

    # run motion command on all particles
    # Reminder: motion -> [velocity, omega]
    def integrate_movement(self, motion):
        for i in range(self._number_of_particles):
            v = random.gauss(motion[0], np.sqrt(self.sigma_motion[0, 0]))
            omega = random.gauss(motion[1], np.sqrt(self.sigma_motion[1, 1]))

            # Add noise to particle pose
            theta = self._particles[i][2]
            self._particles[i][0] += v * self._T * np.cos(theta)  # x
            self._particles[i][1] += v * self._T * np.sin(theta)  # y
            self._particles[i][2] = (self._particles[i][2] + omega * self._T) % (2 * np.pi)  # orientation

        self.world.drawPoints(self._particles)
        return

    # weight all particles with likelihood-field and resample
    #
    # dist_list: Datas or robot sensors (distance to measured point, None if laser measured nothing)
    # alpha_list: Datas of robot sensors (angle to each other from right to left (negative to postive) as radians)
    # distant_map: Type of 'myWorld.getDistanceGrid()'
    def integrate_measurement(self, dist_list, alpha_list, distant_map):

        wheel = []
        weight_sum = 0
        for idx in range(self._number_of_particles): # self._particles: # iterate over pixels and transform robot laser beams
            tmp_weight = self.get_weight_of_particle(self._particles[idx], dist_list, alpha_list, distant_map)
            weight_sum += tmp_weight
            if idx == 0:
                wheel.append(tmp_weight)
            else:
                wheel.append(wheel[idx-1] + tmp_weight)

        self.resample(wheel, weight_sum)
        return


    # Calc weight of a single particle with likely hoodfield
    def get_weight_of_particle(self,particle, dist_list, alpha_list, distant_map):
        weight = 1

        for idx in range(len(dist_list)):

            distance = dist_list[idx]

            if distance is not None:
                x_cord = distance * np.cos(alpha_list[idx] + particle[2]) + particle[0]
                y_cord = distance * np.sin(alpha_list[idx] + particle[2]) + particle[1]

                # self.world.addLineL(partikel[0], partikel[1], partikelBeamX, partikelBeamY)

                hood_value = distant_map.getValue(x_cord, y_cord)
                if hood_value is None:  # If measured point of a particle is negative
                   return 0

                # data = (1/(np.sqrt(2 * np.pi * np.power(0.25, 2)))) * np.power(np.e, -(np.power(x-mean, 2)) / 2 * np.power(0.25, 2))
                weight *= self.norm_distribution(hood_value, 0, 0.4)  # magic number if hood_value is zero

        return weight

    def resample(self, wheel, weight_sum):
        particles_old = self._particles

        # Roulette
        for idx in range(self._number_of_particles):
            r = np.random.uniform(0, weight_sum)

            # Binary search
            # https://en.wikipedia.org/wiki/Binary_search_algorithm
            # https://www.topcoder.com/community/data-science/data-science-tutorials/binary-search/
            left = 0
            right = self._number_of_particles - 1
            while left <= right:
                m = int(left + (right - left)/2)

                if r == wheel[m]:
                    break

                elif r > wheel[m]:
                    left = m + 1
                else:
                    right = m - 1


            self._particles[idx][0] = particles_old[m][0]
            self._particles[idx][1] = particles_old[m][1]
            self._particles[idx][2] = particles_old[m][2]


    def norm_distribution(self, x, mean, sd):
        return np.exp(-(x - mean) ** 2 / (2 * (sd ** 2))) / (2 * np.pi * (sd ** 2)) ** 0.5
        #return (np.exp(-(x - mean) ** 2 / 2 / sd ** 2) / ((np.sqrt(2 * np.pi) * sd)))


    # calc average pose
    def get_pose(self):
        return np.mean(self._particles[3])


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