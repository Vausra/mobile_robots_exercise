# class KalmanFilterPoseEstimator
# O. Bittel; 28.11.2017


from math import *
import numpy as np
import numpy.linalg as la

class KalmanFilterPoseEstimator:

    # --------
    # init: initializes pose = (x,y,orientation) and covariance sigma_pos
    #
    def __init__(self):
        self.x = 0.0 # initial x position
        self.y = 0.0 # initial y position
        self.orientation = pi/2 # initial orientation
        self.sigma_pose =  np.zeros((3,3))
        self.T = 0.1 # time step
        self.landmarks = [] # landmark positions

    # --------
    # print robot attributes.
    #
    def __repr__(self):
        return 'Kalman filter pose: [x=%6.4f y=%6.4f orient=%6.4f]' % (self.x, self.y, self.orientation)

    # --------
    # set time step.
    #
    def setTimestep(self, T):
        self.T = float(T)

    # --------
    # set landmark positions
    #
    def setLandmarks(self, l):
        self.landmarks = l

    # --------
    # sets the initial robot position estimate
    #
    def setInitialPose(self, pose):
        new_x, new_y, new_orientation = pose
        assert new_orientation >= 0 and new_orientation < 2 * pi, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    # --------
    # set the initial position covariance.
    #
    def setInitialCovariance(self, sigma_pose):
        self.sigma_pose = sigma_pose

    # --------
    # get the current robot position estimate.
    #
    def getPose(self):
        return [self.x, self.y, self.orientation]

    # --------
    # get the current position covariance.
    #
    def getCovariance(self):
        return self.sigma_pose

    # --------
    # integrate motion = [v, omega] and covariance sigma_movement for the next time step
    #
    def integrateMovement(self, motion, sigma_motion):
        v = motion[0]
        omega = motion[1]
        # Jacobians of g:
        G_pose = np.array([[ 1, 0, -self.T*v*sin(self.orientation)],
                           [ 0, 1,  self.T*v*cos(self.orientation)],
                           [ 0, 0,  1]])
        G_motion = np.array([[ self.T*cos(self.orientation),      0],
                             [ self.T*sin(self.orientation),      0],
                             [ 0,                            self.T]])
        # Compute new position:
        self.x += v*self.T * cos(self.orientation)
        self.y += v*self.T * sin(self.orientation)
        self.orientation = (self.orientation + omega*self.T) % (2*pi)
        # Compute new covariance:
        # print 'sigma_pose'
        # print self.sigma_pose
        # print 'G_pose * self.sigma_pose * G_pose.T'
        # print G_pose * self.sigma_pose * G_pose.T
        # print 'G_motion * sigma_motion * G_motion.T'
        # print G_motion * sigma_motion * G_motion.T
        self.sigma_pose = G_pose.dot(self.sigma_pose).dot(G_pose.T) + G_motion.dot(sigma_motion).dot(G_motion.T)
        # print 'sigma_pose neu'
        # print self.sigma_pose
        # print


    # --------
    # integrate distance measurements = z with covariance sigma_z
    #
    def integrateMeasurement(self, z, sigma_z):

        n = len(self.landmarks)
        if n == 0:
            return

        # erwarterter Messvektor z_e:
        z_e = []
        for i in range(n):
            dist = (self.x-self.landmarks[i][0])**2
            dist += (self.y-self.landmarks[i][1])**2
            dist = sqrt(dist)
            z_e.append(dist)
        z_e = np.array(z_e)

        # Jacobian of h:
        # Attention: H must be 2D-array
        H = np.array([[(self.x-self.landmarks[0][0])/z_e[0], (self.y-self.landmarks[0][1])/z_e[0], 0]])
        for i in range(1,n):
            H1 = np.array([[(self.x-self.landmarks[i][0])/z_e[i],(self.y-self.landmarks[i][1])/z_e[i], 0]])
            H = np.vstack((H,H1))

        # Kalman gain:
        K = self.sigma_pose.dot(H.T).dot(la.inv(H.dot(self.sigma_pose).dot(H.T) + sigma_z))
        
        # Compute new estimate:
        z = np.array(z)
        kor = K.dot(z-z_e)
        self.x += kor[0]
        self.y += kor[1]
        self.orientation = (self.orientation + kor[2]) % (2*pi)
        
        # Compute new covariance:
        self.sigma_pose = (np.eye(3) - K.dot(H)).dot(self.sigma_pose)


