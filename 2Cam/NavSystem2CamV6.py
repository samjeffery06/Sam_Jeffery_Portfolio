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
import PiClientContinuous2 as pcc2
import ObjectDetectionHierarchy as od
import PathPlanning as pp
import ChaikinSmoothing as cs
import MotorController11PurePursuit as mc
import RPi.GPIO as GPIO

class NavSystem(object):
    def __init__(self):
        self.myClient = None
        self.myClient2 = None
        self.background1 = None
        self.background2 = None
        self.client1active = 1
        self.client2active = 1
        self.backgroundtotal = None
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
        self.AGVcam1Conn1 = []
        self.AGVcam1Both = []
        self.AGVcam2Conn2 = []
        self.AGVcam2Both = []
        self.maxTime = 0
        self.photoTimeTot = 0
        self.notFound = 0
           
        #02/10 Calibration
        self.dist = np.array([[ 7.35896202e+00, -6.66217648e+02, 1.05176443e-01, 1.83536062e-02, 1.95789771e+04]])
        self.mtx = np.array([[2.25635315e+03, 0, 3.25309533e+02], [0, 2.22782604e+03, 2.15060758e+02], [0, 0, 1]])
        
        self.dist2 = np.array([[ 4.70674610e-01, -2.13037413e+01, -3.78506308e-02,  4.98897483e-02, 1.50845089e+02]])
        self.mtx2 = np.array([[1.46559544e+03, 0, 3.19919900e+02],  [0, 1.47528010e+03, 2.40209988e+02], [0, 0, 1]])
        self.findTime = 0
        self.controlTime = 0
        #New Hierachy lets go
        self.photoFlag = 0
        self.done = 0
        self.loopCount = 0
        
        #Cam Boundaries
        self.whichCam = 0        
        self.boundaries = [600, 445],[265,350]

    def setup(self,camno):
        if camno == 0: 
            GPIO.setmode(GPIO.BCM)
        if camno == 0 or camno == 1:
            self.myClient = pcc.getClient()
            print 'TCP client 1 is connected'
        if camno == 0 or camno == 2:
            self.myClient2 = pcc2.getClient()
            print 'TCP client 2 is connected'
            
    def closecam(self,camno):    
        if camno == 2:
            myNavSystem.myClient2.stopImageCapturing()
            myNavSystem.myClient2.closeClient()
            print 'Closed Cam 2'
            
        elif camno == 1:
            myNavSystem.myClient.stopImageCapturing()
            myNavSystem.myClient.closeClient()
            print 'Closed Cam 1'            

    def undistortImg(self, img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
        return dst
    
    def undistortImg2(self, img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx2, self.dist2, (w, h), 1, (w, h))
        dst = cv2.undistort(img, self.mtx2, self.dist2, None, newcameramtx)
        return dst

    def findMarkersCam1(self):
        self.background1 = cv2.imread('Background1.jpeg', 0)
        self.background1 = self.undistortImg(self.background1)
        ret, self.background1 = cv2.threshold(self.background1, 90, 255, cv2.THRESH_BINARY)
        #cv2.imwrite('Games.jpg',self.background)

        img = self.myClient.getImage()
        img = self.undistortImg(img)
        ret, img = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY)

        roi = self.background1 - img

        self.AGVrect, self.theta = od.findObj(roi, 0)
        self.whichCam = 1
        self.destrect, alpha = od.findObj(roi, 1)

        print 'First Camera Checked'
        
    #Added late on 20/08 (21/08 early)    
    def findMarkersCam2(self):
        self.background2 = cv2.imread('Background2.jpeg', 0)
        self.background2 = self.undistortImg2(self.background2)
        ret, self.background2 = cv2.threshold(self.background2, 90, 255, cv2.THRESH_BINARY)
        #cv2.imwrite('Games.jpg',self.background)

        img = self.myClient2.getImage()
        img = self.undistortImg2(img)
        ret, img = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY)

        roi = self.background2 - img
        cv2.imwrite('destImage.jpeg',img)
        cv2.imwrite('roi.jpeg',roi)
        if self.AGVrect == None:
            self.AGVrect, self.theta = od.findObj(roi, 0)
            self.AGVrect, self.theta = self.transform(self.AGVrect, self.theta)
            self.whichCam = 2
            
        if self.destrect == None:
            self.destrect, alpha = od.findObj(roi, 1)
            self.destrect, alpha = self.transform(self.destrect, alpha)

        print 'Second Camera Checked'
        
    #new code #Added late on 20/08 (21/08 early)
    def transform(self, pos, t):
        t = t*math.pi/180
        x1 = 395.3
        y1 = 438.3
        t1 = 94.97*math.pi/180
        x2 = 157.38
        y2 = 300.69
        t2 = -174.81*math.pi/180
        dT = t1 - t2
        
        T = (t + dT)*180/math.pi
        X = x1 + (pos[0][0]-x2)*math.cos(dT) + (pos[0][1]-y2)*math.sin(dT)
        Y = y1 - (pos[0][0]-x2)*math.sin(dT) + (pos[0][1]-y2)*math.cos(dT)
        retval = [[X, Y], [pos[1][0], pos[1][1]]]
        if T > 180:
            T = T - 360
        elif T < -180:
            T = T + 360
            
        return retval,T

    def planPath(self):
        print 'Planning path...'
        start = self.AGVrect[0]
        self.start = (int(round(start[0])), int(round(start[1])))
        end = self.destrect[0]
        self.end = (int(round(end[0])), int(round(end[1])))
        
        self.backgroundtotal = cv2.imread('Background0.jpg',0)
        ret, self.backgroundtotal = cv2.threshold(self.backgroundtotal, 90, 255, cv2.THRESH_BINARY)
        grid = self.backgroundtotal
        pathImg = self.backgroundtotal.copy()

        kernel = np.ones([40, 40])
        kernel2 = np.ones([20, 20])
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
        img2 = self.backgroundtotal.copy()
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
    
    def OpenCloseCams(self):
        print 'Start Opening and closing'
        if self.client1active == 0:
            if self.AGVrect[0][0] < 630 and self.AGVrect[0][1] < 470:
                self.setup(1)
                #time.sleep(1)
                self.myClient.startImageCapturing()
                self.client1active = 1
                for x in range(0,2):
                    print 'open cam1'
        else:
            if self.AGVrect[0][0] > 650 or self.AGVrect[0][1] > 490:
                self.closecam(1)
                self.client1active = 0
                for x in range(0,2):
                    print 'close cam 1'
            
        if self.client2active == 0:
            #print 'getting here'
            if self.AGVrect[0][0] > 240 and self.AGVrect[0][1] > 310:
                self.setup(2)
                #time.sleep(1)
                self.myClient2.startImageCapturing()
                self.client2active = 1
                for x in range(0,2):
                    print 'open cam2'
        else:
            if self.AGVrect[0][0] < 220 or self.AGVrect[0][1] < 290:
                self.closecam(2)
                self.client2active = 0
                for x in range(0,2):
                    print 'close cam2'
        #time.sleep(0.1)
            
    def chooseCam(self):
        if self.AGVrect[0][0] > self.boundaries[0][0] or self.AGVrect[0][1] > self.boundaries[0][1]:
            self.whichCam = 2
        if self.AGVrect[0][0] < self.boundaries[1][0] or self.AGVrect[0][1] < self.boundaries[1][1]:
            #Algorithm for picking next camera... in this situation there is only one option
            self.whichCam = 1
        if self.whichCam == 1:
            print 'Using cam 1'
        else:
            print 'Using cam 2'
        
            
    def stopLoop(self):
        if self.client1active == 1:
            myNavSystem.myClient.stopImageCapturing()
            self.myClient.closeClient()
        
        if self.client2active == 1:
            myNavSystem.myClient2.stopImageCapturing()
            self.myClient2.closeClient()
            
        self.myVehicle.steer(0, 0)
        GPIO.cleanup() #added this patch... disappeared a while back
        img3 = self.backgroundtotal.copy()
        pp.drawPath(self.pixelPath, img3)
        for i in range(len(self.AGVpath)):
            cv2.circle(img3, self.AGVpath[i], 2, 0, 1)
            
        for i in range(len(self.AGVpos)):
            cv2.circle(img3, self.AGVpos[i], 1, 0, 1)
        
        cv2.imwrite('StopLoop.jpg',self.backgroundtotal)
        resultimg = cv2.imread('StopLoop.jpg')
        #ret, resultimg = cv2.threshold(resultimg, 90, 255, cv2.THRESH_BINARY)
        pp.drawPath(self.pixelPath, resultimg)
        for i in range(len(self.AGVcam1Conn1)):
            #cv2.circle(resultimg, self.AGVcam1Conn1[i], 1, (255, 0, 0), 4)
            cv2.circle(resultimg, self.AGVcam1Conn1[i], 2, 0, 1)
        
        for i in range(len(self.AGVcam1Both)):
            #cv2.circle(resultimg, self.AGVcam1Both[i], 1, (255, 0, 0), 4)
            #cv2.circle(resultimg, self.AGVcam1Both[i], 4, (0, 0, 255), 2)
            cv2.circle(resultimg, self.AGVcam1Both[i], 2, 0, 1)
        
        for i in range(len(self.AGVcam2Both)):
            #cv2.circle(resultimg, self.AGVcam2Both[i], 1, (0, 0, 255), 4)
            #cv2.circle(resultimg, self.AGVcam2Both[i], 4, (255, 0, 0), 2)
            cv2.circle(resultimg, self.AGVcam2Both[i], 2, 0, 1)
            
        for i in range(len(self.AGVcam2Conn2)):
            #cv2.circle(resultimg, self.AGVcam2Conn2[i], 1, (0, 0, 255), 4)
            cv2.circle(resultimg, self.AGVcam2Conn2[i], 2, 0, 1)
    
        cv2.imwrite('Actualpath.jpeg', img3)
        cv2.imwrite('CamPath.jpeg', resultimg)
        print 'saved the thing'
            
    def photoLoop(self):
        while not self.done:
            pickCam = self.whichCam #So if whichCam updates mid stream there aren't issues #Added late on 20/08 (21/08 early)
            photoTime = time.time() #new
            self.loopCount = self.loopCount + 1
            #Added late on 20/08 (21/08 early)
            if pickCam == 1:
                try:
                    img5 = self.myClient.getImage()
                    #cv2.imwrite('photo' + str(self.loopCount) + '.jpeg', img5)
                    img5 = self.undistortImg(img5)
                    print 'Photo from cam 1'
                except:
                    img5 = self.background1
            else:
                try:
                    img5 = self.myClient2.getImage()
                    #cv2.imwrite('photo' + str(self.loopCount) + '.jpeg', img5)
                    img5 = self.undistortImg2(img5)
                    print 'Photo from cam 2'
                except:
                    img5 = self.background2
            
            ret, img5 = cv2.threshold(img5, 90, 255, cv2.THRESH_BINARY)
            self.AGVrectCam, self.thetaCam = od.findObj(img5, 0)
            print 'Photo Loop Time '+ str(time.time() - photoTime) #new
            if self.AGVrectCam:  #!= None:
                #Added late on 20/08 (21/08 early)
                if pickCam == 2: #for multiple cameras can implement clever algos to limit number of if statements
                    #(only check surrounding cameras not all)
                    self.AGVrectCam, self.thetaCam = self.transform(self.AGVrectCam, self.thetaCam)
                    
                if pickCam == 1:
                    if self.client2active == 1:
                        self.AGVcam1Both.append((int(round(self.AGVrectCam[0][0])), int(round(self.AGVrectCam[0][1]))))
                    else:
                        self.AGVcam1Conn1.append((int(round(self.AGVrectCam[0][0])), int(round(self.AGVrectCam[0][1]))))
                else:
                    if self.client1active == 1:
                        self.AGVcam2Both.append((int(round(self.AGVrectCam[0][0])), int(round(self.AGVrectCam[0][1]))))
                    else:
                        self.AGVcam2Conn2.append((int(round(self.AGVrectCam[0][0])), int(round(self.AGVrectCam[0][1]))))
                time.sleep(0.5)
                self.photoFlag = 1
                self.findTime = photoTime
                self.photoTimeTot = self.photoTimeTot + time.time()-photoTime
            else:
                self.notFound = 1
                self.loopCount = self.loopCount - 1
       
 
    
if __name__ == '__main__':

    myNavSystem = NavSystem()
    myNavSystem.setup(0)
    time.sleep(3)
    myNavSystem.myClient.startImageCapturing()
    myNavSystem.myClient2.startImageCapturing()
    markerTime = time.time()
    myNavSystem.findMarkersCam1()
    myNavSystem.findMarkersCam2()
    myNavSystem.OpenCloseCams()
    myNavSystem.chooseCam()
    
    print 'markerTime: ' + str(time.time() - markerTime)
    pathTime = time.time()
    myNavSystem.planPath()
    myNavSystem.pathSmoothing(3)
    pathplanT = time.time() - pathTime
    
    myNavSystem.controllerSetup()
    
    
    myNavSystem.done = 0
    camTime = 0
    camAgg = 0
    camCount = 0
    update = 0
    upX = 0
    upY = 0
    upTh = 0
    totX = 0
    totY = 0
    totTh = 0
    keepTime = 0
    loopTimes = 0
    loopStart = 0
    left = 0
    right = 0
    pos = 0
    face = 0
    dist = 0
    distmax = 0
    startTime = time.time()
    th1 = threading.Thread(target=myNavSystem.photoLoop)
    th1.daemon = True
    th1.start()
    loopStart = time.time()+1
    while not myNavSystem.done:
        if myNavSystem.notFound == 1:
            totX = 0
            totY = 0
            totTh = 0
            keepTime = 0
            myNavSystem.notFound = 0
        
        if myNavSystem.photoFlag == 1:
            myNavSystem.AGVrect, myNavSystem.theta = myNavSystem.forwardEstimator(left, right, time.time()-loopStart, myNavSystem.AGVrectCam, myNavSystem.thetaCam, totX, totY, totTh) #updated
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
            print 'Forward Estimator Integrals'
            print totX, totY, totTh
            keepTime = keepTime + time.time()-loopStart
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
            if dist > distmax:
                distmax = dist
            temp = time.time()
            myNavSystem.chooseCam()
            myNavSystem.OpenCloseCams()
            camAgg = camAgg + time.time() - temp
            camCount = camCount + 1
            if (time.time() - temp) > camTime:
                camTime = time.time() - temp
                
            print myNavSystem.AGVrect[0]
            print myNavSystem.client2active
            #print myNavSystem.theta
            update = 0
        time.sleep(0.005)    
        
        
    myNavSystem.myController.vehicle.steer(0, 0)
    myNavSystem.stopLoop()
    
    print 'Path planning took ' + str(pathplanT) + ' seconds'
    print 'Average Photo Time ', str(myNavSystem.photoTimeTot/myNavSystem.loopCount)
    print myNavSystem.loopCount
    print 'Total Time ', (time.time() - startTime)
    print 'Max Dist from path ', distmax
    print 'Max Dealing with camera time ', camTime
    print 'Average time for cam ', str(camAgg/camCount)
    print 'Cam Count ', camCount 
    print 'AGV has arrived at destination'
    print 'Program will terminate...'
