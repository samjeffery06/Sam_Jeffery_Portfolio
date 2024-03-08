# This is the main file that calls all other modules for navigation
# Author: S. Brits
# Std nr: 17573459
# October 2016

import cv2
import numpy as np
import threading
import time
import PiClientContinuous as pcc
import ObjectDetectionHierarchy as od
import PathPlanning as pp
import ChaikinSmoothing as cs
import MotorControllerOld as mc
import RPi.GPIO as GPIO

class NavSystem(object):
    def __init__(self):
        self.myClient = None
        self.background = None
        self.AGVrect = None
        self.theta = None
        self.destrect = None
        self.start = None
        self.end = None
        self.grid = None
        self.centerPoints = None
        self.pixelPath = None
        self.myVehicle = None
        self.myController = None
        self.AGVpath = []
        self.maxTime = 0
        self.notFound = 0
        self.distance = 0
        self.distcount = 0
        self.distmax = 0
        self.dist = np.array([[ 4.70674610e-01, -2.13037413e+01, -3.78506308e-02,  4.98897483e-02, 1.50845089e+02]])
        self.mtx = np.array([[1.46559544e+03, 0, 3.19919900e+02],  [0, 1.47528010e+03, 2.40209988e+02], [0, 0, 1]])
        #self.dist = np.array([[ 1.29543667e+00, -2.14725923e+02, -1.95436707e-02, -6.85882084e-02, 5.59432374e+03]])
        #self.mtx = np.array([[2.44278715e+03, 0, 3.19227899e+02], [0, 2.57317495e+03, 2.39482830e+02], [0, 0, 1]])
        self.findTime = 0
        self.controlTime = 0

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
        img2 = self.background.copy()
        for i in range(iterations):
            self.centerPoints = cs.smoothing(self.centerPoints, 0.65)  # Chaikin path smoothing
        self.pixelPath = pp.drawPath(self.centerPoints, img2)  # Final path as pixels
        cv2.imwrite('smoothedPath.jpeg', img2)

    def controllerSetup(self):
        self.myVehicle = mc.Vehicle()
        self.myController = mc.Controller(self.myVehicle, self.pixelPath)
        print 'Controller created...'

    def doLoopControl(self,img2):
        loopStart = time.time()
        img2 = self.undistortImg(img2)
        cv2.imwrite('LoopPhoto.jpeg',img2)
        ret, img2 = cv2.threshold(img2, 90, 255, cv2.THRESH_BINARY)
        self.AGVrect, self.theta = od.findObj(img2, 0)
        self.findTime += time.time() - loopStart
        print 'find AGV time: ' + str(time.time() - loopStart)

        done = 0
        if self.AGVrect:
            self.AGVpath.append((int(round(self.AGVrect[0][0])), int(round(self.AGVrect[0][1]))))
            controlTime = time.time()
            time.sleep(0.2)
            done, dist = self.myController.control(self.AGVrect[0], self.theta)
            if dist > self.distmax:
                self.distmax = dist
            print dist
            self.distance = self.distance + dist
            self.distcount = self.distcount + 1
            self.controlTime += time.time() - controlTime
            print 'control time: ' + str(time.time() - controlTime)
        else:
            self.myController.vehicle.steer(0,0)
        return done

    def stopLoop(self):
        self.myClient.closeClient()
        self.myVehicle.steer(0, 0)
        GPIO.cleanup()
        img3 = self.background.copy()
        pp.drawPath(self.pixelPath, img3)
        for i in range(len(self.AGVpath)):
            cv2.circle(img3, self.AGVpath[i], 2, 0, 1)

        cv2.imwrite('Actualpath.jpeg', img3)
        print 'saved the thing'

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
    img = myNavSystem.background.copy()
    cv2.imwrite('unsmoothedPath.jpeg', img)
    myNavSystem.pathSmoothing(3)
    pptime = time.time() - pathTime
    print 'Path planning took ' + str(pptime) + ' seconds'

    myNavSystem.controllerSetup()
    done = 0

    # Control loop
    overallTime = time.time()
    i = 0
    loopTimes = 0
    while not done:
        i += 1
        startTime = time.time()
        photo = myNavSystem.myClient.getImage()
        #cv2.imwrite('photo' + str(i) + '.jpeg', photo)
        done = myNavSystem.doLoopControl(photo)
        loopTimes += time.time() - startTime
        print 'whole loop time: ' + str(time.time() - startTime) + ' s'

    myNavSystem.myController.vehicle.steer(0, 0)
    myNavSystem.myClient.stopImageCapturing()
    myNavSystem.stopLoop()

    print 'Total Time: ', time.time()-overallTime
    print 'Path planning took ' + str(pptime) + ' seconds'
    print 'average control time: ' + str(myNavSystem.controlTime/100)
    print 'average find time: ' + str(myNavSystem.findTime/100)
    print 'average total loop time: ' + str(loopTimes/100)
    print 'average distance from path: ' + str(myNavSystem.distance/myNavSystem.distcount)
    print 'Max Distance from path: ', myNavSystem.distmax
    print 'AGV has arrived at destination'
    print 'Program will terminate...'
