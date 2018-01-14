# class SensorUtilities.
#
# This class defines some utility methods for sensor data.
#
# O. Bittel
# AIN V1.0; 05.11.2017


from math import *
from graphics import *
import World


# --------
# Extracts lines from sensor data dists and directions (polar coordinates).
# A polyline must be supported by a sequence of at least
# minPoints = 3 sensor data points.
#
def extractLinesFromSensorData(dists, directions, eps=0.1):
    minPoints = 3
    numberedPointList = []
    nr = -1
    for i in range(len(dists)):
        d = dists[i]
        if d is None:
            continue
        alpha = directions[i]
        x_l = d * cos(alpha)
        y_l = d * sin(alpha)
        nr += 1
        numberedPointList.append([x_l, y_l, nr])

    extractedNumberedPointList = extractPolyLine(numberedPointList, eps)
    if len(extractedNumberedPointList) == 0:
        return []

    #print('polyline:',extractedNumberedPointList)

    listOfExtractedLines = []
    for i in range(1,len(extractedNumberedPointList)):
        point1 = extractedNumberedPointList[i-1]
        point2 = extractedNumberedPointList[i]
        if point2[2]-point1[2]+1 >= minPoints:
            listOfExtractedLines.append([point1[0:2],point2[0:2]])
    #print('extracted lines:', listOfExtractedLines)
    return listOfExtractedLines


# --------
# Extract a polyline from a list of numbered points.
# https://de.wikipedia.org/wiki/Douglas-Peucker-Algorithmus
#
def extractPolyLine(numberedPointList, eps = 0.1):
    n = len(numberedPointList)
    if n <= 2:
        return numberedPointList

    # Finde Punkt mit groesstem Abstand:
    dmax = 0
    imax = 0
    p1 = Point(numberedPointList[0][0], numberedPointList[0][1])
    p2 = Point(numberedPointList[n - 1][0], numberedPointList[n - 1][1])
    l = Line(p1,p2)
    for i in range(1, len(numberedPointList)):
        p = Point(numberedPointList[i][0], numberedPointList[i][1])
        d = World.World.distPointSegment(p,l)
        if d > dmax:
            dmax = d
            imax = i

    # Splitte, falls dmax > eps:
    if dmax > eps:
        res1 = extractPolyLine(numberedPointList[0:imax + 1], eps)
        res2 = extractPolyLine(numberedPointList[imax:n], eps)
        return res1+res2[1:]
    else:
        return [numberedPointList[0], numberedPointList[n - 1]]


# --------
# Transform several polylines from robot's local to global coordinates
#
def transform(polylines, pose):
    x_R,y_R,theta_R = pose
    polylines_g = []
    for polyline in polylines:
        polyline_g = []
        for p in polyline:
            x = p[0]
            y = p[1]
            x_g = x_R + x*cos(theta_R) - y*sin(theta_R)
            y_g = y_R + x*sin(theta_R) + y*cos(theta_R)
            polyline_g.append([x_g,y_g])
        polylines_g.append(polyline_g)
    return polylines_g


if __name__ == "__main__":
    points = [[0,0],[1,0.05],[2,0.2],[3,0]]
    print(points)
    lines = extractPolyLine(points)
    print(lines)


