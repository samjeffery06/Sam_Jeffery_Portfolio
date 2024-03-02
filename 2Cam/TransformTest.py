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
import MotorController10 as mc
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
        self.AGVrect2 = None
        self.theta2 = None
        self.AGVrectCam = None
        self.thetaCam = None
        self.destrect = None
        self.destrect2 = None
        self.start = None
        self.end = None
        self.grid = None
        self.centerPoints = None
        self.pixelPath = None
        self.myVehicle = None
        self.myController = None
        self.AGV = [] #record actual position of the AGV
        self.AGVpos = [] #record position estimator guess of position
        self.AGVcam = []
        self.AGVcam1Conn1 = []
        self.AGVcam1Both = []
        self.AGVcam2Conn2 = []
        self.AGVcam2Both = []
        self.maxTime = 0
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

        self.AGVrect2, self.theta2 = od.findObj(roi, 0)
        self.AGV.append((int(round(self.AGVrect2[0][0])), int(round(self.AGVrect2[0][1]))))

        print 'AGV Location'
        print self.AGVrect2
        print 'Angle: ' + str(self.theta2)
        
        
    #Added late on 20/08 (21/08 early)    
    def findMarkersCam2(self):
        self.background2 = cv2.imread('Background2.jpeg', 0)
        self.background2 = self.undistortImg2(self.background2)
        ret, self.background2 = cv2.threshold(self.background2, 90, 255, cv2.THRESH_BINARY)
        #cv2.imwrite('backreplace.jpg',self.background2)

        img = self.myClient2.getImage()
        img = self.undistortImg2(img)
        ret, img = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY)

        roi = self.background2 - img
        #cv2.imwrite('destImage.jpeg',img)
        #cv2.imwrite('roi.jpeg',roi)
        self.AGVrect, self.theta = od.findObj(roi, 0)
        print 'AGV Location no trans'
        print self.AGVrect
        print 'Angle: ' + str(self.theta)
        self.AGVrect, self.theta = self.transform(self.AGVrect, self.theta)
        self.AGV.append((int(round(self.AGVrect[0][0])), int(round(self.AGVrect[0][1]))))


        print 'From Camera 2'
        print 'AGV Location'
        print self.AGVrect
        print 'Angle: ' + str(self.theta)

        
    #new code #Added late on 20/08 (21/08 early)
    def transform(self, pos, t):
        t = t*math.pi/180
        x1 = 353.82
        y1 = 425.26
        t1 = -157.17*math.pi/180
        x2 = 123.2
        y2 = 347.5
        t2 = -68.20*math.pi/180
        dT = t1 - t2
        T = (t + dT)*180/math.pi
        X = x1 + (pos[0][0]-x2)*math.cos(dT) + (pos[0][1]-y2)*math.sin(dT)
        Y = y1 - (pos[0][0]-x2)*math.sin(dT) + (pos[0][1]-y2)*math.cos(dT)
        retval = [[X, Y], [pos[1][0], pos[1][1]]]
        
        return retval,T

    
    def straightDist(self,point1,point2):
        dx = (point1[0] - point2[0])
        dy = (point1[1] - point2[1])
        dist = (dx**2+dy**2)**0.5
        return dist    
    
    
    def stopLoop(self):
        
        myNavSystem.myClient.stopImageCapturing()
        self.myClient.closeClient()
        
        myNavSystem.myClient2.stopImageCapturing()
        self.myClient2.closeClient()
            
        #self.myVehicle.steer(0, 0)
        #GPIO.cleanup() #added this patch... disappeared a while back


    
        
        cv2.imwrite('StopLoop.jpg',self.background1)
        img3 = cv2.imread('StopLoop.jpg')
        
        cv2.circle(img3, self.AGV[0], 1, (255, 0, 0), 3)
        cv2.circle(img3, self.AGV[1], 1, (0, 0, 255), 3)
        
    
        cv2.imwrite('Comparison.jpeg', img3)

        print 'saved the thing'
            
    
       
 
    
if __name__ == '__main__':

    myNavSystem = NavSystem()
    myNavSystem.setup(0)
    #time.sleep(3)
    myNavSystem.myClient.startImageCapturing()
    myNavSystem.myClient2.startImageCapturing()
    
    imagetest = myNavSystem.myClient.getImage()
    imagetest2 = myNavSystem.myClient2.getImage()
    #cv2.imwrite('PhotoCam1.jpeg',imagetest)
    cv2.imwrite('Pos4Cam1.jpeg',imagetest)
    cv2.imwrite('Pos4Cam2.jpeg',imagetest)
    # Image processing and path planning
    markerTime = time.time()
    myNavSystem.findMarkersCam2()
    myNavSystem.findMarkersCam1()
    dist = myNavSystem.straightDist(myNavSystem.AGVrect[0],myNavSystem.AGVrect2[0])
    print 'Transform Accuracy: ',dist
    dist = myNavSystem.straightDist([353.82000732421875, 425.260009765625],myNavSystem.AGVrect2[0])
    print 'Distance from calibration point: ',dist
    myNavSystem.stopLoop()
    
    
    