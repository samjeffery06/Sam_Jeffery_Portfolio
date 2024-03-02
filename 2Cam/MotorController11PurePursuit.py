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
        bigF = 200
        carD = 146/4.7586
        
        #Pure Pursuit Path Following
        closestPoint,index, dist = self.closestPoint(currPos)
        pathLength = len(self.path)
        end = self.path[len(self.path)-1]
        distanceToEnd = self.straightDist(currPos,end)
        if distanceToEnd > 40:
            if pathLength-index > 30:
                lookaheadPoint = self.path[index+30]
            else:
                lookaheadPoint = end
            D = self.straightDist(currPos,lookaheadPoint)
            #print 'D'
            #print D
            lookaheadAngle = self.findAngle(currPos,lookaheadPoint)
            print 'Index ', index
            print 'Angle'
            print lookaheadAngle
            print currAngle
            diff = lookaheadAngle - currAngle
            if diff > 180:
                diff = diff - 360
            elif diff < -180:
                diff = diff + 360
            z = (90 - (diff))*math.pi/180
            print 'Z'
            print z
            if z < 0 and face == 0:
                R = bigF
                L = bigF/5
            elif z > math.pi and face == 0:
                L = bigF
                R = bigF/5
            else:
                face = 1
                print 'Z'
                print z
                xgv = D*math.cos(z)
                #xgv = (lookaheadPoint[0]-currPos[0])*math.cos(currAngle)-(lookaheadPoint[1]-currPos[1])*math.sin(currAngle)
                #print 'xgv'
                #print xgv
                if xgv == 0:
                    Y = 1000000
                else:
                    Y = (D**2)/(2*xgv)
                #print 'Y'
                #print Y
                if Y > 0:
                    if Y < carD:
                        ratio = 3
                    else:
                        ratio = (2/(((2*Y)/carD)-1))+1
                    R = bigF
                    L = bigF/ratio
                else:
                    if Y > -carD:
                        ratio = 3
                    else:
                        ratio = (2/(((2*(-Y))/carD)-1))+1
                    L = bigF
                    R = bigF/ratio
            #print 'Ratio'
            #print ratio
            print R, L
            self.vehicle.steer(R,L)  #both wheels forward movement for the moment
        else:
            done = 1
            print 'You have reached your destination'
        return done, L, R, dist, face
