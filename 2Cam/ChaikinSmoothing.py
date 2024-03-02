# Chaikin smoothing function
# Author: S. Brits
# Std nr: 17573459
# October 2016

import numpy as np
import cv2
import PathPlanning as pp

def smoothing(points,cut=0.75,closed=0):
    newPoints = []
    if closed:
        pass
    else:
        newPoints.append(points[0])
        for i in range(1,len(points)-1):
            newPoint1 = (int(round(cut*points[i][0]+(1-cut)*points[i-1][0])),int(round(cut*points[i][1]+(1-cut)*points[i-1][1])))
            newPoints.append((newPoint1))
            newPoint2 = (int(round(cut*points[i][0]+(1-cut)*points[i+1][0])),int(round(cut*points[i][1]+(1-cut)*points[i+1][1])))
            newPoints.append((newPoint2))
        newPoints.append(points[len(points)-1])
    return newPoints
