
import math
import ObjectDetectionHierarchy as od
import cv2




def positionEstimator(LeftF, RightF, dt, currPos, currAngle):
        currAngle = currAngle*math.pi/180
        wheelr = 32/4.7586
        carD = 146/4.7586
        if LeftF == RightF:
            Rmid = 10000000
        else:
            Rmid = (LeftF*carD)/(RightF-LeftF) + carD/2
        L1 = math.pi*LeftF*wheelr*dt/100
        L2 = math.pi*RightF*wheelr*dt/100
        
        if LeftF == RightF:        
            Rg = L1
            dTh = 0
        else:
            dTh = (L1+L2)/(2*Rmid)
            Rg = 2*Rmid*math.sin(dTh/2)
            
        Th1 = (currAngle + dTh)*180/math.pi
        #dx = x + Rg*math.cos(currAngle + dTh/2)
        #dy = y - Rg*math.sin(currAngle + dTh/2)
        dx = currPos[0][0] + Rg*math.cos(currAngle + dTh/2)
        dy = currPos[0][1] + Rg*math.sin(currAngle + dTh/2)
        pos = [[dx, dy], [currPos[1][0], currPos[1][1]]]
        return pos, Th1
    
img = cv2.imread(roi.jpeg, 0)
AGVrect, theta = od.findObj(img, 0)
print (AGVrect[0])
posM, resth = positionEstimator(200,200,1,AGVrect, theta)

print (posM[0])
print (resth)