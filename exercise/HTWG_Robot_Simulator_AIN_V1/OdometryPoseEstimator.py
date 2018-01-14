# class OdometryPoseEstimator
# O. Bittel; 26.09.2017

from math import *
import numpy as np

class OdometryPoseEstimator:

    # --------
    # init: initializes pose = (x,y,orientation) and covariance sigma_pose
    #
    def __init__(self):
        self.x = 0.0 # initial x position
        self.y = 0.0 # initial y position
        self.orientation = pi/2 # initial orientation
        self.sigma_pose =  np.zeros((3,3))
        self.T = 0.1 # time step

    # --------
    # print robot attributes.
    #
    def __repr__(self):
        return 'Robot odometry pose: [x=%6.4f y=%6.4f orient=%6.4f]' % (self.x, self.y, self.orientation)

    # --------
    # set time step.
    #
    def setTimestep(self, T):
        self.T = float(T)

    # --------
    # sets the initial robot pose estimate
    #
    def setInitialPose(self, pose):
        new_x, new_y, new_orientation = pose
        assert new_orientation >= 0 and new_orientation < 2 * pi, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    # --------
    # set the initial pose covariance.
    #
    def setInitialCovariance(self, sigma_pose):
        self.sigma_pose = sigma_pose

    # --------
    # get the current robot pose estimate.
    #
    def getPose(self):
        return [self.x, self.y, self.orientation]

    # --------
    # get the current pose covariance.
    #
    def getCovariance(self):
        return self.sigma_pose

    # --------
    # integrate motion = [v, omega] and covariance sigma_movement for the next time step
    #
    def integrateMovement(self, motion, sigma_motion):
        v = motion[0]
        omega = motion[1]

        # Jacobians of f:
        F_pos = np.array([[ 1, 0, -self.T*v*sin(self.orientation)],
                           [ 0, 1,  self.T*v*cos(self.orientation)],
                           [ 0, 0,  1]])
        F_motion = np.array([[ self.T*cos(self.orientation),      0],
                              [ self.T*sin(self.orientation),      0],
                              [ 0,                            self.T]])
        # Compute new position:
        self.x += v*self.T * cos(self.orientation)
        self.y += v*self.T * sin(self.orientation)
        self.orientation = (self.orientation + omega*self.T) % (2*pi)

        # Compute new covariance:
        self.sigma_pose = F_pos.dot(self.sigma_pose).dot(F_pos.T) + F_motion.dot(sigma_motion).dot(F_motion.T)





