# Motor controller module
# Author: S. Brits
# Std nr: 17573459
# October 2016

import RPi.GPIO as GPIO
import math

class Vehicle(object):
    def __init__(self):
        GPIO.setup(26, GPIO.OUT)  # R step
        GPIO.setup(19, GPIO.OUT)  # R dir
        GPIO.setup(13, GPIO.OUT)  # L step
        GPIO.setup(12, GPIO.OUT)  # L dir

        GPIO.output(19, True)
        GPIO.output(12, False)

        self.RightWheel = GPIO.PWM(26, 50)
        self.LeftWheel = GPIO.PWM(13, 50)

    def steer(self, R, L):
        if R:
            self.RightWheel.ChangeFrequency(R)
            self.RightWheel.start(50)
        else:
            self.RightWheel.stop()
        if L:
            self.LeftWheel.ChangeFrequency(L)
            self.LeftWheel.start(50)
        else:
            self.LeftWheel.stop()

class Controller(object):
    def __init__(self,vehicle,path):
        self.vehicle = vehicle
        self.path = path
        self.kD = 180/60
        self.kA = 1

    def straightDist(self,point1,point2):
        dx = (point1[0] - point2[0])
        dy = (point1[1] - point2[1])
        dist = (dx**2+dy**2)**0.5
        return dist

    def findAngle(self,point1,point2):
        dx = point2[0] - point1[0]
        dy = point1[1] - point2[1]

        if dx == 0:
            if dy >=0:
                theta = 90
            else:
                theta = -90
        else:
            theta = math.atan(abs(dy) / abs(dx))
            theta = theta*180/math.pi
            if dx > 0:
                if dy < 0:
                    theta = -theta
            else:
                if dy > 0:
                    theta = 180 - theta
                else:
                    theta = theta -180

        return theta

    def closestPoint(self, currPos):
        dist = self.straightDist(currPos,self.path[0])
        index = 0
        for i in range(len(self.path)):
            distI = self.straightDist(currPos,self.path[i])
            if dist > distI:
                dist = distI
                index = i
        return self.path[index],index, dist

    def squareDistance(self, currPos, closestPoint, lookaheadPoint):
        deltaX = lookaheadPoint[0] - closestPoint[0]
        if deltaX == 0:
            deltaX = 0.1
        ak = (lookaheadPoint[1]-closestPoint[1])/(deltaX)
        bk = -1
        ck = closestPoint[1] - closestPoint[0]*(lookaheadPoint[1]-closestPoint[1])/(deltaX)
        squareD = abs(ak*currPos[0] + bk*currPos[1] + ck)/(ak**2+bk**2)**0.5
        return squareD

    def control(self,currPos,currAngle, face):
        done = 0
        L = 0
        R = 0
        # Follow-the-carrot path following
        closestPoint,index, dist = self.closestPoint(currPos)
        pathLength = len(self.path)
        end = self.path[len(self.path)-1]
        distanceToEnd = self.straightDist(currPos,end)
        if distanceToEnd > 40:
            if pathLength-index > 30:
                lookaheadPoint = self.path[index+30]
                lookaheadAngle = self.findAngle(currPos,lookaheadPoint)
                vehErrorAngle = lookaheadAngle - currAngle

                if vehErrorAngle > 180:
                    vehErrorAngle = vehErrorAngle - 360
                if vehErrorAngle < -180:
                    vehErrorAngle = 360 + vehErrorAngle

                if vehErrorAngle >= 0:
                    L = 110 + vehErrorAngle
                    R = 70 - 0.4*vehErrorAngle
                    #L = 150 + vehErrorAngle
                    #R = 110 - 0.4*vehErrorAngle
                else:
                    R = 110 + abs(vehErrorAngle)
                    L = 70 - 0.4*abs(vehErrorAngle)
                    #R = 150 + abs(vehErrorAngle)
                    #L = 110 - 0.4*abs(vehErrorAngle)

                R = max(0,R)
                L = max(0,L)
                self.vehicle.steer(L,R)  #both wheels forward movement for the moment
        else:
            done = 1
            print 'You have reached your destination'
        return done, R, L, dist, face
