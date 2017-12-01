import numpy as np
import numpy.matlib as mat

'''liefert eine 2D-Rotationsmatrix mit Drehwinkel theta zurueck.'''
def rot(theta):
    return np.array([[mat.cos(mat.radians(theta)), -mat.sin(mat.radians(theta))],
             [mat.sin(mat.radians(theta)), mat.cos(mat.radians(theta))]])


'''liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drechachse x zurueck.'''
def rotx(theta):
    return np.array([[1 ,0 ,0],
            [0, mat.cos(theta), -mat.sin(theta)],
            [0, mat.sin(theta), mat.cos(theta)]])


'''liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drechachse y zurueck'''
def roty(theta):
    return np.array([[mat.cos(theta), 0, mat.sin(theta)],
            [0, 1, 0],
            [-mat.sin(theta), 0, mat.cos(theta)]])


'''liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drechachse z zurueck.'''
def rotz(theta):
    return np.array([[mat.cos(theta), -mat.sin(theta), 0],
             [mat.sin(theta), mat.cos(theta), 0],
             [0, 0, 1]])


'''wandelt die Rotationsmatrix r in eine homogene Transformationsmatrix um und liefert diese zurueck.'''
def rot2trans(r):
    y,x = np.shape(r)
    id = np.identity(y+1)
    id[0:y, 0:x] = r
    return id


'''liefert eine homogene Translationsmatrix mit Translation t zurueck.
t ist ein Tupel der Groesse 2 bzw. 3 fuer den 2D bzw. 3D Fall.'''
def trans(t):
    y,x = np.shape(t)
    id = np.identity(y+1)
    id[0:y, y:] = t

    return id

if __name__ == "__main__":
    print("rotation 2D ", rot(90))
    print("rotation 3D x", rotx(90))
    print("rotation 3D y", roty(90))
    print("rot to trans 3D z", rot2trans(rotx(90)))
    print("trans ", trans(np.array([[3], [4]])))
    print("trans ", trans(np.array([[3],[4],[5]])))

    #rot2trans(rot(90))
