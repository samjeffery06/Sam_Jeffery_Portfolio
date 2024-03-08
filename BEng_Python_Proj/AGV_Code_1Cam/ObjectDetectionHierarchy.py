# Object detection using contour hierarchy module
# Author: S. Brits
# Std nr: 17573459
# October 2016

import cv2
import numpy as np

def numberOfChildren(hI,child,n,children):
    n += 1
    children.append(child)
    if hI[child][0] == -1:
        pass
    else:
        n,children = numberOfChildren(hI,hI[child][0],n,children)
    return n,children

def findObj(img,obj):
    imgno, contoursI, hI = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    hI = hI[0]

    parents = []
    for i in range(len(contoursI)):
        if hI[i][2] == -1:
            pass
        else:
            parents.append(i)

    if obj == 0:
        one = 0
        two = 0
        three = 0
        for i in range(len(parents)):
            childs = []
            number, children = numberOfChildren(hI, hI[parents[i]][2], 0, childs)
            one = 0
            two = 0
            three = 0
            if number == 3:
                Pchildren = []
                for a in children:
                    if hI[a][2] == -1:
                        pass
                    else:
                        Pchildren.append(a)
                for j in Pchildren:
                    childs2 = []
                    number2, children2 = numberOfChildren(hI, hI[j][2], 0, childs2)
                    hasChildren = 0
                    for k in children2:
                        if hI[k][2] == -1:
                            pass
                        else:
                            hasChildren = 1
                    if hasChildren:
                        break
                    if number2 == 1:
                        one = 1
                        frontContourTemp = j
                    if number2 == 2:
                        two = 1
                    if number2 == 3:
                        three = 1
                    # print [one, two, three]
                    if one and two and three:
                        break
                if one and two and three: #first
                    frontContour = frontContourTemp
                    break
                else:
                    one = 0
                    two = 0
                    three = 0
        if one and two and three:
            print 'Found the AGV'
            # find centroid of matched contour
            M = cv2.moments(contoursI[frontContour])
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            rectangle = cv2.minAreaRect(contoursI[parents[i]])
            width = rectangle[1][0]
            height = rectangle[1][1]
            centerX = rectangle[0][0]
            centerY = rectangle[0][1]
            flip = 0

            if cX < centerX:
                flip = 1

            if width > height:
                theta = -rectangle[2]
                if flip:
                    theta -= 180
            else:
                theta = -rectangle[2] - 90
                if flip:
                    theta += 180

            if theta == 90:
                if cY > centerY:
                    theta = -90
        else:
            print 'Didnt find the AGV'
            rectangle = None
            theta = None
            #cv2.imwrite('NotFoundDest.jpeg', img)

    elif obj == 1:
        one = 0
        two = 0
        three = 0
        for i in range(len(parents)):
            childs = []
            number, children = numberOfChildren(hI, hI[parents[i]][2], 0, childs)
            one = 0
            two = 0
            if number == 2:
                # print 'two children, parent ' + str(parents[i])
                Pchildren = []
                for a in children:
                    if hI[a][2] == -1:
                        pass
                    else:
                        Pchildren.append(a)
                for j in Pchildren:
                    childs2 = []
                    number2, children2 = numberOfChildren(hI, hI[j][2], 0, childs2)
                    hasChildren = 0
                    for k in children2:
                        if hI[k][2] == -1:
                            pass
                        else:
                            hasChildren = 1
                    if hasChildren:
                        break
                    if number2 == 1:
                        one = 1
                    if number2 == 2:
                        two = 1
                    if one and two:
                        break
                if one and two:
                    break
                else:
                    one = 0
                    two = 0
        if one and two:
            print 'Found the destination'
            rectangle = cv2.minAreaRect(contoursI[parents[i]])
            theta = 0
        else:
            print 'Didnt find the destination'
            rectangle = None
            theta = None
            cv2.imwrite('NotFoundDest.jpeg',img)

    return rectangle, theta