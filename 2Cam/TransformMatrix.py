
import math



def transform(camno,pos,t):
    t = t*math.pi/180
    x1 = 566
    y1 = 405
    t1 = math.pi/2
    x2 = 64
    y2 = 239
    t2 = 3*math.pi/4
    dT = t1 - t2
    T = (t + dT)*180/math.pi
    X = x1 + (pos[0][0]-x2)*math.cos(dT) + (pos[0][1]-y2)*math.sin(dT)
    Y = y1 - (pos[0][0]-x2)*math.sin(dT) + (pos[0][1]-y2)*math.cos(dT)
    retval = [[X, Y], [pos[1][0], pos[1][1]]]
    
    return retval,T


position, outT = transform(0,271,416,0)
print (position[0])
print (outT*180/math.pi)