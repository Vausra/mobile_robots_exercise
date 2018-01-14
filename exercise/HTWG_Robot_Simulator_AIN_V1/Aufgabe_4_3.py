# Aufgabe 4.3
# O. Bittel; 31.10.2017

from PlotUtilities import *
import numpy as np
import numpy.linalg as la
import random as rd

# --------
# generate n position particles following the normal distribution
# N(position, sigma_position)
#
def generateParticleSet(position, sigma_position, n):
    assert sigma_position.shape == (2,2), 'sigma_pose should be 2*2 matrix'
    assert len(position) == 2, 'pos must be of the form [x,y]'

    ax = position[0] - 3*sqrt(sigma_position[0,0])
    bx = position[0] + 3*sqrt(sigma_position[0,0])
    ay = position[1] - 3*sqrt(sigma_position[1,1])
    by = position[1] + 3*sqrt(sigma_position[1,1])
    c = ndf(position,position,sigma_position)
    particles = []
    i = 0
    while i < n:
        x = ax + (bx-ax)*rd.random()
        y = ay + (by-ay)*rd.random()
        q = c*rd.random()
        p = ndf([x,y],position,sigma_position)
        if q < p:
            i += 1
            particles.append([x,y])
    return particles


def ndf(x,mu,Sigma):
    n = len(x)
    c = 1/sqrt((2*pi)**n * la.det(Sigma))
    SigmaI = la.inv(Sigma)
    #return c*np.exp(-0.5 * (x-mu).T * Sigma.I * (x-mu))
    return c*np.exp(-0.5 * (x-mu).dot(SigmaI).dot(x-mu))


def getMeanCovFromParticleSet(particles):
    x = np.array(range(len(particles)))
    y = np.array(range(len(particles)))
    i = 0
    for p in particles:
        x[i] = p[0]
        y[i] = p[1]
        i += 1
    mean = [np.mean(x),np.mean(y)]
    #cov = ml.matrix(np.cov(x,y))
    cov = np.array(np.cov([x,y]))
    return mean, cov


def aufg_4_3():

    # Generiere und plotte Partikelmenge:
    Sigma = np.array([
        [56.63, 20.25],
        [20.25,  8.37]])
    mu = np.array([4,3])
    particles = generateParticleSet(mu, Sigma, 500)
    plotPositionParticles(particles)


    # Vergleiche Sigma, mu mit Mittelwert und Kovarianz der Partikelmenge:
    plotPositionCovariance(mu,Sigma,'r')
    particles_mean, particles_cov = getMeanCovFromParticleSet(particles)
    print(particles_mean)
    print(particles_cov)
    plotPositionCovariance(particles_mean,particles_cov,'k')


    # Wende auf Partikelmenge Systemmodell an:
    A = np.array([
        [0, -2],
        [2,  0]])
    b = np.array([10,10])

    for i in range(len(particles)):
        x = np.array(particles[i])
        y = A.dot(x) + b
        particles[i] = y


    # Plotte neue Partikelmenge mit Kovarianz:
    plotPositionParticles(particles,'g')
    plotPositionCovariance(A*mu+b,A*Sigma*A.T,'r')
    particles_mean, particles_cov = getMeanCovFromParticleSet(particles)
    plotPositionCovariance(particles_mean,particles_cov,'k')

    plotShow()


aufg_4_3()
