#construction of veronoi

import numpy as np
from vision import Vision

class PathSet:
    paths = np.array([])

    def AddPath(self,NewPath):
        np.append(paths,NewPath)

    def VisializePath(self):


class Triangle:

    def __init__(self,vertices):
        self.vertices = vertices




class Voronoi:
    #define the initialize function of Voronoi
    #MinPoint = [-4600,3000]
    #MaxPoint = [4600,3000]
    Delaunay = Array()
    VoroPaths = PathSet()
    TempTriangles

    def __init__(self,points):
        self.points = points
        #sort the points

    def __ConstructDelaunay(self):

    def __FindSuperTriangle(self):
        [xmin,ymin] = np.amin(self.points,0)
        [xmax,ymax] = np.amax(self.points,0)
        TempPointM = np.array([(xmin+xmax)/2, (xmin-xmax)/2+ymin])
        TempPointL = np.array([-ymax+xmin+ymin,ymax])
        TempPointR = np.array([])

    def 
