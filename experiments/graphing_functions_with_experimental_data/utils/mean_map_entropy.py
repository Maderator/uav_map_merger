import sys, os
import random
import copy
import numpy as np
import math

def meanMapEntropy(cloud, radius=0.3):

    radius_sq = radius**2
    me = []

    for point in cloud:
        pts_in_radius = np.array(getPtsInRadius(point, cloud, radius_sq))
        cov = np.cov(pts_in_radius.transpose())
        det = np.linalg.det(2*math.pi*math.e*cov)
        if det > 0:
            log = 1.0/2.0*math.log(det)
            me.append(log)

    return np.mean(me)



def distSq(p1, p2):
    return (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2

def getPtsInRadius(p, cloud, r_sq):

    pts = []

    for pt in cloud: 
        if distSq(pt, p) < r_sq:
            pts.append(pt[0:2])
    
    return pts
        
