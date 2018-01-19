import numpy as np
import numpy.linalg as la
import random
import Particle
from HTWG_Robot_Simulator_AIN_V1 import World


class ParticleFilterPoseEstimator:

    # noise says how strong the random noise for each particle should be
    def __init__(self, noise):

        # protected variables
        self._sigma_motion = np.zeros((3, 3))
        self._T = 0.1  # time step
        self._particles = []
        self._number_of_particles = 0
        self._covariance = 0
        self._noise = noise
        self._world = None

    # create random particles in area (pose_from, pose_to)
    def initialize(self, pose_from, pose_to, n=200):
        #assert pose_from >= 0, 'pose_from should be greater or equal to 0'
        #assert pose_to <= 20, 'pose_to mus be lower or equal to 20'

        self._number_of_particles = n

        for i in range(n):

            #orientation = np.pi * random.random()
            #position_x = random.random() * (pose_to[0] - pose_from[0]) + pose_from[0]
            #position_y = random.random() * (pose_to[1] - pose_from) + pose_from

            position_x = random.uniform(pose_from[0], pose_to[0])
            position_y = random.uniform(pose_from[1], pose_to[1])
            orientation = random.uniform(pose_from[2], pose_to[2])

            self._particles.append([position_x, position_y, orientation, 0])

        return self._particles

    def get_particles(self):
        return self._particles

    def set_timestep(self, T):
        self._T = T

    def set_world(self, world):
        self._world = world

    # run motion command on all particles
    # Reminder: motion -> [velocity, omega]
    '''def integrate_movement(self, motion):
        v = motion[0]
        omega = motion[1]
        sigma_noise = (self._k_theta / self._T) * abs(omega)  # turning rate noise

        for i in range(len(self._particles)):
            v = random.gauss(motion[0], np.sqrt(sigma_motion[0, 0]))
            omega = random.gauss(motion[1], np.sqrt(sigma_motion[1, 1]))
            # Apply noisy motion to particle _particles[i]:
            theta = self._particles[i][2]
            self._particles[i][0] += v * self.T * np.cos(theta)  # x
            self._particles[i][1] += v * self.T * np.sin(theta)  # y
            self._particles[i][2] = (self._particles[i][2] + omega * self.T) % (2 * np.pi)  # orientation

        #self.sigma_pose = F_pos.dot(self.sigma_pose).dot(F_pos.T) + F_motion.dot(sigma_motion).dot(F_motion.T)
        return
'''
    # weight all particles with likelihoodfield-algorythm and resample
    #
    # dist_list:
    # distant_map: Type of 'myWorld.getDistanceGrid()'
    def integrated_measurement(self, dist_list, alpha_list, distant_map):
        assert type(World) is distant_map, 'distant map has to be a type of the call World.getDistanceGrird()'

        #resampling()

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

    def calcCordsFromDistance(self, senseData):
        estimations = []
        for index, s in np.ndenumerate(senseData):
            if s is not None:
                grad = (index[0] * 10) + 45
                x_cord = s * np.cos(np.radians(grad))
                y_cord = s * np.sin(np.radians(grad))
                estimations.append([x_cord, y_cord, 0])

        return estimations