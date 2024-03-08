# This is the main file that calls all other modules for navigation
# Author: S. Brits, S Jeffery
# Std nr: 17573459
# October 2016

import cv2
import numpy as np
import threading
import time
import math
import PiClientContinuous as pcc
import ObjectDetectionHierarchy as od
import PathPlanning as pp
import ChaikinSmoothing as cs
#import MotorController11ModCarrot as mc
import MotorController11PurePursuit as mc
#import MotorController10 as mc
import RPi.GPIO as GPIO

class NavSystem(object):
    def __init__(self):
        self.myClient = None
        self.background = None
        self.AGVrect = None
        self.theta = None
        self.AGVrectCam = None
        self.thetaCam = None
        self.destrect = None
        self.start = None
        self.end = None
        self.grid = None
        self.centerPoints = None
        self.pixelPath = None
        self.myVehicle = None
        self.myController = None
        self.AGVpath = [] #record actual position of the AGV
        self.AGVpos = [] #record position estimator guess of position
        self.AGVcam = []
        self.AGVcamOff = []
        self.maxTime = 0
        self.notFound = 0
        self.distance = 0
        self.distcount = 0
        self.phototime = 0
        self.numphotos = 0
        
        #Calibration Cam1
        #self.dist = np.array([[ 7.35896202e+00, -6.66217648e+02, 1.05176443e-01, 1.83536062e-02, 1.95789771e+04]])
        #self.mtx = np.array([[2.25635315e+03, 0, 3.25309533e+02], [0, 2.22782604e+03, 2.15060758e+02], [0, 0, 1]])
        
        #Calibration cam2
        self.dist = np.array([[ 4.70674610e-01, -2.13037413e+01, -3.78506308e-02,  4.98897483e-02, 1.50845089e+02]])
        self.mtx = np.array([[1.46559544e+03, 0, 3.19919900e+02],  [0, 1.47528010e+03, 2.40209988e+02], [0, 0, 1]])
        
        self.findTime = 0
        self.controlTime = 0
        #New Hierachy lets go
        self.photoFlag = 0
        #REMOVEDself.estFlag = 0
        #REMOVEDself.loopTime = 0
        self.done = 0
        self.loopCount = 0
        #REMOVEDself.posCount = 0
        #Motor Frequencies
        #REMOVEDself.LeftF = 0
        #REMOVEDself.RightF = 0

    def setup(self):
        GPIO.setmode(GPIO.BCM)
        self.myClient = pcc.getClient()
        print 'TCP client is connected'

    def undistortImg(self, img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
        return dst

    def findMarkers(self):
        self.background = cv2.imread('Background0.jpeg', 0)
        self.background = self.undistortImg(self.background)
        ret, self.background = cv2.threshold(self.background, 90, 255, cv2.THRESH_BINARY)

        img = self.myClient.getImage()
        img = self.undistortImg(img)
        ret, img = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY)

        roi = self.background - img

        self.AGVrect, self.theta = od.findObj(roi, 0)
        self.destrect, alpha = od.findObj(roi, 1)

        print 'AGV and destination markers found...'

    def planPath(self):
        print 'Planning path...'
        start = self.AGVrect[0]
        self.start = (int(round(start[0])), int(round(start[1])))
        end = self.destrect[0]
        self.end = (int(round(end[0])), int(round(end[1])))

        grid = self.background[:]
        pathImg = self.background.copy()

        kernel = np.ones([40, 40])
        kernel2 = np.ones([20,20])
        self.grid = cv2.erode(grid, kernel)  # Make obstacles larger to account for AGV size
        self.grid = cv2.erode(self.grid, kernel2)
        cv2.imwrite('grid.jpeg', self.grid)

        self.grid = pp.makeSquareGrid1024(self.grid)  # Make grid square for quadtree algorithm
        tree = pp.Tree(self.grid)  # Create quadtree
        quad = tree.drawTree()
        cv2.imwrite('quad.jpeg', quad)
        graph = pp.aStar(tree, self.start, self.end, quad)  # Create graph from tree
        graph.Solve()  # Find path
        self.centerPoints = graph.getPathCenterPoints()  # Get center points of path nodes
        self.pixelPath = pp.drawPath(self.centerPoints,pathImg)               # Connect nodes to form path
        print 'Path found...'

    def pathSmoothing(self,iterations):
        img2 = self.background.copy()
        for i in range(iterations):
            self.centerPoints = cs.smoothing(self.centerPoints, 0.65)  # Chaikin path smoothing
        self.pixelPath = pp.drawPath(self.centerPoints, img2)  # Final path as pixels
        cv2.imwrite('smoothedPath.jpeg', img2)

    def controllerSetup(self):
        self.myVehicle = mc.Vehicle()
        self.myController = mc.Controller(self.myVehicle, self.pixelPath)
        print 'Controller created...'

    
    def positionEstimator(self, LeftF, RightF, dt, currPos, currAngle):
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
        print 'Dtheta from pos est'
        print dTh
        
        Th1 = (currAngle + dTh)*180/math.pi
        dxout = Rg*math.cos(currAngle + dTh/2)
        dyout = - Rg*math.sin(currAngle + dTh/2)
        dx = currPos[0][0] + dxout
        dy = currPos[0][1] + dyout
        pos = [[dx, dy], [currPos[1][0], currPos[1][1]]]
        return pos, Th1, dxout, dyout, dTh
    
    def forwardEstimator(self, LeftF, RightF, dt, currPos, currAngle, dxin, dyin, dThin):
        currAngle = currAngle*math.pi/180 + dThin
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
        dx = currPos[0][0] + Rg*math.cos(currAngle + dTh/2) + dxin
        dy = currPos[0][1] - Rg*math.sin(currAngle + dTh/2) + dyin
        pos = [[dx, dy], [currPos[1][0], currPos[1][1]]]
        return pos, Th1
    
    def stopLoop2(self):
        self.myClient.closeClient()
        self.myVehicle.steer(0, 0)
        GPIO.cleanup() #added this patch... disappeared a while back
        img3 = self.background.copy()
        pp.drawPath(self.pixelPath, img3)
        for i in range(len(self.AGVpath)):
            cv2.circle(img3, self.AGVpath[i], 2, 0, 1)
            
        for i in range(len(self.AGVpos)):
            cv2.circle(img3, self.AGVpos[i], 1, 0, 1)
        
        cv2.imwrite('StopLoop.jpg',self.background)
        img31 = cv2.imread('StopLoop.jpg')
        #ret, resultimg = cv2.threshold(resultimg, 90, 255, cv2.THRESH_BINARY)
        pp.drawPath(self.pixelPath, img31)
        for i in range(len(self.AGVcam)):
            cv2.circle(img31, self.AGVcam[i], 2, (255, 0, 0), 2)
            
        for i in range(len(self.AGVcamOff)):
            cv2.circle(img31, self.AGVcamOff[i], 2, (0, 0, 255), 2) 
        
        cv2.imwrite('Actualpath.jpeg', img3)
        cv2.imwrite('CamPath.jpeg', img31)
        print 'saved the thing'
    
    def stopLoop(self):
        self.myClient.closeClient()
        self.myVehicle.steer(0, 0)
        GPIO.cleanup() #added this patch... disappeared a while back
        img3 = self.background.copy()
        pp.drawPath(self.pixelPath, img3)
        for i in range(len(self.AGVpath)):
            cv2.circle(img3, self.AGVpath[i], 2, 0, 1)
            
        for i in range(len(self.AGVpos)):
            cv2.circle(img3, self.AGVpos[i], 1, 0, 1)
        
        img31 = self.background.copy()
        pp.drawPath(self.pixelPath, img31)
        for i in range(len(self.AGVcam)):
            cv2.circle(img31, self.AGVcam[i], 2, 0, 1)
        
        cv2.imwrite('ActualpathNoPosEst.jpeg', img3)
        cv2.imwrite('CamPathNoPosEst.jpeg', img31)
        print 'saved the thing'
   
            
    def straightDist(self,point1,point2):
        dx = (point1[0] - point2[0])
        dy = (point1[1] - point2[1])
        dist = (dx**2+dy**2)**0.5
        return dist
    
    def closestPoint(self, currPos):
        dist = self.straightDist(currPos,self.pixelPath[0])
        index = 0
        for i in range(len(self.pixelPath)):
            distI = self.straightDist(currPos,self.pixelPath[i])
            if dist > distI:
                dist = distI
                index = i
        return dist
    
    def photoLoop(self):
        while not self.done:
            photoTime = time.time() 
            self.loopCount = self.loopCount + 1
            try:
                img5 = self.myClient.getImage()
                #cv2.imwrite('photo' + str(self.loopCount) + '.jpeg', img5)
            except:
                img5 = self.background
            
            img5 = self.undistortImg(img5)
            ret, img5 = cv2.threshold(img5, 90, 255, cv2.THRESH_BINARY)
            self.AGVrectCam, self.thetaCam = od.findObj(img5, 0)
            print 'Photo Loop Time '+ str(time.time() - photoTime)
            self.phototime = self.phototime + time.time() - photoTime
            self.numphotos = self.numphotos + 1
            if self.AGVrectCam:  #!= None:
                self.AGVcam.append((int(round(self.AGVrectCam[0][0])), int(round(self.AGVrectCam[0][1]))))
                time.sleep(0.2)
                self.photoFlag = 1
                self.findTime = photoTime
                print 'Photo in ' + str(self.AGVrectCam[0]) + 'Angle:' + str(self.thetaCam)
                #if (self.loopCount < 5 or self.loopCount > 20) and (self.loopCount < 35 or self.loopCount > 55):
                #    self.photoFlag = 1
                #    self.AGVcam.append((int(round(self.AGVrectCam[0][0])), int(round(self.AGVrectCam[0][1]))))
                #    print 'Getting Feedback'
                #else:
                #    self.AGVcamOff.append((int(round(self.AGVrectCam[0][0])), int(round(self.AGVrectCam[0][1]))))
                #    print 'No feedback'
                #self.findTime = photoTime
            else:
                self.notFound = 1
                
 
    
if __name__ == '__main__':

    myNavSystem = NavSystem()
    myNavSystem.setup()
    time.sleep(3)
    myNavSystem.myClient.startImageCapturing()
    
    # Image processing and path planning
    markerTime = time.time()
    myNavSystem.findMarkers()
    print 'markerTime: ' + str(time.time() - markerTime)
    pathTime = time.time()
    myNavSystem.planPath()
    myNavSystem.pathSmoothing(3)
    print 'Path planning took ' + str(time.time() - pathTime) + ' seconds'

    myNavSystem.controllerSetup()
    
    #Local Variable definitions
    myNavSystem.done = 0
    update = 0
    upX = 0
    upY = 0
    upTh = 0
    totX = 0
    totY = 0
    totTh = 0
    distmax = 0
    keepTime = 0
    loopTimes = 0
    loopStart = 0
    left = 0
    right = 0
    pos = 0
    face = 0
    startTime = time.time()
    t1 = threading.Thread(target=myNavSystem.photoLoop)
    t1.daemon = True
    t1.start()
    loopStart = time.time()+1
    distpic = 0
    
    #Control loop starting
    while not myNavSystem.done:
        if myNavSystem.notFound == 1:
            totX = 0
            totY = 0
            totTh = 0
            keepTime = 0
            myNavSystem.notFound = 0
            
        if myNavSystem.photoFlag == 1:
            tempdist = myNavSystem.closestPoint(myNavSystem.AGVrectCam[0])
            if tempdist > distpic:
                distpic = tempdist
            myNavSystem.AGVrect, myNavSystem.theta = myNavSystem.forwardEstimator(left, right, time.time()-loopStart, myNavSystem.AGVrectCam, myNavSystem.thetaCam, totX, totY, totTh) #updated
            #myNavSystem.AGVrect = myNavSystem.AGVrectCam
            #myNavSystem.theta = myNavSystem.thetaCam
            totX = 0
            totY = 0
            totTh = 0
            keepTime = keepTime + time.time()-loopStart
            print 'Photo Loop time: ' + str(time.time()-myNavSystem.findTime) + ' Sum of Loops: ' + str(keepTime)
            
            keepTime = 0
            myNavSystem.photoFlag = 0
            update = 1
            
        elif time.time()-loopStart > 0.2:
            myNavSystem.AGVrect, myNavSystem.theta, upX, upY, upTh = myNavSystem.positionEstimator(left, right, time.time()-loopStart, myNavSystem.AGVrect, myNavSystem.theta) #updated
            totX = totX + upX
            totY = totY + upY
            totTh = totTh + upTh
            print 'Changes: '
            print totX, totY, totTh
            keepTime = keepTime + time.time()-loopStart
            print 'Used posEstimator ' + str(time.time()-loopStart)
            pos = 1
            update = 1
                
        if update == 1:
            loopStart = time.time()
            if pos == 1:
                myNavSystem.AGVpos.append((int(round(myNavSystem.AGVrect[0][0])), int(round(myNavSystem.AGVrect[0][1])))) 
            else:
                myNavSystem.AGVpath.append((int(round(myNavSystem.AGVrect[0][0])), int(round(myNavSystem.AGVrect[0][1]))))
            pos = 0
            myNavSystem.done, left, right, dist, face = myNavSystem.myController.control(myNavSystem.AGVrect[0], myNavSystem.theta, face)
            print dist
            if dist > distmax:
                distmax = dist
            myNavSystem.distance = myNavSystem.distance + dist
            myNavSystem.distcount = myNavSystem.distcount + 1
            print myNavSystem.AGVrect[0]
            print myNavSystem.theta
            update = 0
            print 'Left: ' + str(left) + ' Right: ' + str(right)
        time.sleep(0.1005)    
        
        
    print 'whole loop time: ' + str(time.time() - startTime) + ' s'
        

    myNavSystem.myController.vehicle.steer(0, 0)
    myNavSystem.myClient.stopImageCapturing()
    myNavSystem.stopLoop()

    #REMOVEDprint 'average control time: ' + str(myNavSystem.controlTime/100)
    #REMOVEDprint 'average find time: ' + str(myNavSystem.findTime/100)
    #REMOVEDprint 'average total loop time: ' + str(loopTimes/100)
    print 'average distance from path: ' + str(myNavSystem.distance/myNavSystem.distcount)
    print 'NBNBmax distance from path: ' + str(distpic)
    print 'Average photo time: ' + str(myNavSystem.phototime/myNavSystem.numphotos)
    print 'Total photos: ' + str(myNavSystem.numphotos)
    print 'AGV has arrived at destination'
    print 'Program will terminate...'
