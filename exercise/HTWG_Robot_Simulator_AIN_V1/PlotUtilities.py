# Funktion zum Plotten von Kovarianzen, Partikelwolken
# und Trajektorien.
# O. Bittel; 26.09.2017

from math import *
import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
import time


# --------
# plot position and sigma ellipse
#
def plotPositionCovariance(position, sigma_position, color = 'b'):
    assert sigma_position.shape == (2,2), 'sigma_pose should be 2*2 matrix'
    assert len(position) == 2, 'position must be of the form [x,y]'

    # Falls sigma_position nahe bei (0,0; 0,0), dann Problem mit Singularitaet
    if sigma_position[0,0] < 1.0E-10 or sigma_position[1,1] < 1.0E-10 :
        return;
    d, V = la.eig(la.inv(sigma_position))

    # Nicht-rotierte Ellipse:
    gamma = np.linspace(0,2*pi,80)
    xp = np.sin(gamma)/sqrt(d[0])
    yp = np.cos(gamma)/sqrt(d[1])

    # Ellipse rotieren und verschieben:
    Xp = np.vstack((xp,yp))
    Xp = V.dot(Xp)
    xp = Xp[0,:] + position[0]
    yp = Xp[1,:] + position[1]
    plt.plot(xp,yp,color+'-')
    plt.plot(position[0],position[1],color+'o')
    return


# --------
# plot position and sigma ellipse and
# and plot orientation as cone.
#
def plotPoseCovariance(pose, sigma_pose, color = 'b'):
    assert sigma_pose.shape == (3,3), 'sigma_pose should be 3*3 matrix'
    assert len(pose) == 3, 'pose must be of the form [x,y,theta]'

    # Ellipse plotten:
    plotPositionCovariance(pose[0:2], sigma_pose[0:2,0:2], color)

    # Zeichne Ausrichtung und sigma als Trichter:
    theta = pose[2]
    sigma_theta = sqrt(sigma_pose[2,2])
    d = sqrt(sigma_pose[0,0] + sigma_pose[1,1]) # Trichterlaenge
    theta1 = theta - sigma_theta/2
    theta2 = theta + sigma_theta/2
    x0 = pose[0]
    y0 = pose[1]
    x1 = x0 + d*cos(theta1)
    y1 = y0 + d*sin(theta1)
    x2 = x0 + d*cos(theta2)
    y2 = y0 + d*sin(theta2)
    plt.plot((x0,x1),(y0,y1),color+'-')
    plt.plot((x0,x2),(y0,y2),color+'-')
    return


# --------
# All plotting must be completed be plotShow()
#
def plotShow():
    plt.axis('equal')
    plt.grid()
    #plt.show()
    plt.show(block=False)
    #time.sleep(2)
    #plt.close()
    return


# --------
# plot a trajectory.
# poses is a list of poses or positions.
#
def plotPositions(poses, color = 'b'):
    x = []
    y = []
    for p in poses:
        x.append(p[0])
        y.append(p[1])
    plt.plot(x,y,color+'-')
    return


# --------
# plot pose particles with orientation
#
def plotPoseParticles(poseList, l = 0.1, color = 'b'):
    x = [0,0]
    y = [0,0]
    for p in poseList:
        x[0] = p[0]
        y[0] = p[1]
        theta = p[2]
        x[1] = x[0] + l*cos(theta)
        y[1] = y[0] + l*sin(theta)
        plt.plot(p[0],p[1],color+'.')
        plt.plot(x,y,color+'-')
    return

# --------
# plot position particles without orientation.
# chi is a list of poses or positions.
#
def plotPositionParticles(posList, color = 'b'):
    for p in posList:
        plt.plot(p[0],p[1],color+'.')
    return

def test1():
    Sigma1 = np.array([[56.63, 20.25],
                       [20.25, 8.37]])
    #Sigma1 = np.array([[48.2500, -27.2798],
    #                   [-27.2798, 16.7500]])

    x1 = np.array([2, 1])
    plotPositionCovariance(x1, Sigma1)

    Sigma2 = np.array([[0.69, -1.25],
                       [-1.25, 3.56]])

    x2 = np.array([4, 3])
    plotPositionCovariance(x2, Sigma2)

    S1i = la.inv(Sigma1)
    S2i = la.inv(Sigma2)
    Sigma = la.inv(S1i + S2i)
    x = Sigma.dot(S1i.dot(x1) + S2i.dot(x2))
    plotPositionCovariance(x, Sigma, 'r')

    # Nicht vergessen:
    plotShow()

def test2():
    Sigma1 = np.array([[0.25, 0.00],
                       [0.00, 4.00]])
    theta1 = pi / 4
    s = sin(theta1)
    c = cos(theta1)
    V = np.array([[c, -s],
                  [s, c]])
    Sigma1r = la.inv((V.dot(la.inv(Sigma1)).dot(V.T)))
    print(Sigma1)

    x = np.array([0, 0])
    plotPositionCovariance(x, Sigma1)
    plotPositionCovariance(x, Sigma1r)

    Sigma2 = np.array([[64.00, 0.00],
                       [0.00, 1.00]])
    theta2 = pi / 6
    s = sin(theta2)
    c = cos(theta2)
    V = np.array([[c, -s],
                  [s, c]])
    Sigma2r = la.inv((V.dot(la.inv(Sigma2)).dot(V.T)))
    print(Sigma2)

    x = np.array([0, 0])
    plotPositionCovariance(x, Sigma2, 'r')
    plotPositionCovariance(x, Sigma2r, 'r')

    plotShow()

if __name__ == "__main__":
    test1()




