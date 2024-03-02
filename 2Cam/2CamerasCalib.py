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
        self.background1 = None
        self.background2 = None
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
        #REMOVEDself.estFlag = 0
        #REMOVEDself.loopTime = 0
        self.done = 0
        self.loopCount = 0
        #REMOVEDself.posCount = 0
        #Motor Frequencies
        #REMOVEDself.LeftF = 0
        #REMOVEDself.RightF = 0

#    def setup(self):
#        GPIO.setmode(GPIO.BCM)
#        self.myClient = pcc.getClient()
#        print 'TCP client 1 is connected'
#        self.myClient2 = pcc2.getClient()
#        print 'TCP client 2 is connected'

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

    def findMarkers(self):
        self.background1 = cv2.imread('Background1.jpeg', 0)
        self.background1 = self.undistortImg(self.background1)
        ret, self.background1 = cv2.threshold(self.background1, 90, 255, cv2.THRESH_BINARY)
        cv2.imwrite('piece1.jpg',self.background1)
        
        self.background2 = cv2.imread('Background2.jpeg', 0)
        self.background2 = self.undistortImg2(self.background2)
        ret, self.background2 = cv2.threshold(self.background2, 60, 255, cv2.THRESH_BINARY)
        cv2.imwrite('piece2.jpg',self.background2)

        img = cv2.imread('Marker1.jpeg', 0)
        img = self.undistortImg(img)
        cv2.imwrite('UndistortMarker1.jpg',img)
        ret, img = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY)

        roi = self.background1 - img

        self.AGVrect, self.theta = od.findObj(roi, 0)
        self.destrect, angle = od.findObj(roi,1)
        
        print 'image 1'
        print self.AGVrect, self.theta
        print self.destrect, angle
        
        img = cv2.imread('Marker2.jpeg', 0)
        img = self.undistortImg2(img)
        cv2.imwrite('UndistortMarker2.jpg',img)
        ret, img = cv2.threshold(img, 90, 255, cv2.THRESH_BINARY)

        roi = self.background2 - img

        self.AGVrect, self.theta = od.findObj(roi, 0)
        self.destrect, angle = od.findObj(roi,1)
        
        print 'image 2'
        print self.AGVrect, self.theta
        print self.destrect, angle
        


        print 'AGV and destination markers found...'

    


            
    
if __name__ == '__main__':

    myNavSystem = NavSystem()
    myNavSystem.findMarkers()



    print 'Program will terminate...'
