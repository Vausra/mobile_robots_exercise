# Aufgabe 4.1
# O. Bittel; 31.10.2017

from PlotUtilities import *

Sigma1 = np.array([[56.63, 20.25],
                   [20.25, 8.37]])
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

