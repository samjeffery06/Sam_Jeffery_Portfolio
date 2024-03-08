# Path planning module using modified A* algorithm
# Author: S. Brits
# Std nr: 17573459
# October 2016

import cv2
import numpy as np
from Queue import PriorityQueue

class Node(object):
    def __init__(self, parent, type, center = None, halfwidth = 0):
        self.type = type
        self.neighbours = None
        self.NW = None
        self.NE = None
        self.SW = None
        self.SE = None
        self.obstacleNode = False

        if parent == None:
            self.parent = None
            self.center = center[:]
            self.halfwidth = halfwidth
        else:
            self.parent = parent
            self.halfwidth = parent.halfwidth/2
            if type == 'NW':
                self.center = (parent.center[0] - parent.halfwidth / 2, parent.center[1] - parent.halfwidth / 2)
            if type == 'NE':
                self.center = (parent.center[0] + parent.halfwidth / 2, parent.center[1] - parent.halfwidth / 2)
            if type == 'SW':
                self.center = (parent.center[0] - parent.halfwidth / 2, parent.center[1] + parent.halfwidth / 2)
            if type == 'SE':
                self.center = (parent.center[0] + parent.halfwidth / 2, parent.center[1] + parent.halfwidth / 2)

    def createChildren(self):
        self.NW = Node(self, 'NW')
        self.NE = Node(self, 'NE')
        self.SW = Node(self, 'SW')
        self.SE = Node(self, 'SE')

    def getNeighbours(self, root):
        self.neighbours = []
        self.recursiveNeighbour(root)

    def recursiveNeighbour(self, root):
        if not (self.intersect(root)):
            return
        if (root.NW == None):
            if root.obstacleNode == False:
                if not (self.center == root.center):
                    self.neighbours.append(root)
            return
        self.recursiveNeighbour(root.NW)
        self.recursiveNeighbour(root.NE)
        self.recursiveNeighbour(root.SW)
        self.recursiveNeighbour(root.SE)

    def intersect(self, node):
        dx = min(self.center[0] + self.halfwidth + 1, node.center[0] + node.halfwidth) - max(
            self.center[0] - self.halfwidth - 1, node.center[0] - node.halfwidth - 1)
        dy = min(self.center[1] + self.halfwidth + 1, node.center[1] + node.halfwidth) - max(
            self.center[1] - self.halfwidth - 1, node.center[1] - node.halfwidth - 1)

        if (dx >= 0) and (dy >= 0):
            return True
        return False

    def doesContain(self,point):
        if ((point[0] >= self.center[0]-self.halfwidth) and (point[0] <= self.center[0] + self.halfwidth)) and ((point[1] >= self.center[1]-self.halfwidth) and (point[1] <= self.center[1] + self.halfwidth)):
            return True
        return False


class Tree(object):
    def __init__(self, grid):
        self.root = Node(None,0,(len(grid)/2,len(grid)/2), len(grid)/2)
        self.grid = grid

        self.buildTree(self.root)

    def buildTree(self,root):
        obstacles = self.checkObst(root)
        if obstacles:
            root.createChildren()
            self.buildTree(root.NW)
            self.buildTree(root.NE)
            self.buildTree(root.SW)
            self.buildTree(root.SE)

    def checkObst(self,node):
        obstacle = False
        number = np.mean(self.grid[node.center[1]-node.halfwidth:node.center[1]+node.halfwidth,node.center[0]-node.halfwidth:node.center[0]+node.halfwidth])
        number = np.mean(number)
        if number==0:   #maybe change
            node.obstacleNode = True
        elif (number>0 and number <255):
            obstacle = True
        return obstacle

    def drawTree(self):
        img = np.zeros((len(self.grid),len(self.grid[0])),dtype = np.uint8)
        self.count = 0
        self.drawNode(self.root, img)
        print 'Number of nodes: ' + str(self.count)
        return img

    def drawNode(self,node,img):
        if node.NW == None:
            if node.obstacleNode==False:
                self.count+=1
            colour = 100
            if node.type == 'NW':
                colour = 160
            elif node.type == 'NE':
                colour = 140
            elif node.type == 'SW':
                colour = 120
            if node.obstacleNode==True:
                colour = 0
            img[node.center[1]-node.halfwidth:node.center[1]+node.halfwidth,node.center[0]-node.halfwidth:node.center[0]+node.halfwidth]=colour
        else:
            self.drawNode(node.NW,img)
            self.drawNode(node.NE,img)
            self.drawNode(node.SW,img)
            self.drawNode(node.SE,img)

    def getPointNode(self,point):
        pointNode = self.recursivePointNode(point,self.root)
        return pointNode

    def recursivePointNode(self, point, root):
        if not (root.doesContain(point)):
            return None
        if root.NW == None:
            return root
        result = self.recursivePointNode(point,root.NW)
        if result:
            return result
        result = self.recursivePointNode(point,root.NE)
        if result:
            return result
        result = self.recursivePointNode(point,root.SW)
        if result:
            return result
        result = self.recursivePointNode(point,root.SE)
        if result:
            return result
        return None

def makeSquareGrid(grid2):
    grid = grid2[:]
    if len(grid)>len(grid[0]):
        filler = np.zeros(len(grid)-len(grid[0]))
        for i in range(len(grid)-1):
            grid[i] = np.concatenate((grid[i],filler))
    elif len(grid)<len(grid[0]):
        filler = np.zeros(len(grid[0]))
        for i in range(len(grid[0])-len(grid)):
            grid = np.append(grid,[filler],axis=0)
    return grid

def makeSquareGrid1024(grid2):
    grid = grid2[:]
    grid = np.lib.pad(grid,((0,1024-len(grid)),(0,1024-len(grid[0]))),'constant',constant_values=(0))
    return grid

def getDist(node1, node2):
    dx = abs(node1.center[0] - node2.center[0])
    dy = abs(node1.center[1] - node2.center[1])

    dist = (dx ** 2 + dy ** 2) ** 0.5
    return dist

def get_line(start, end):
    #Bresenham's Line Algorithm - RogueBasin. 2016. Bresenham's Line Algorithm - RogueBasin. [ONLINE] Available at: http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm. [Accessed 09 June 2016].
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    blocked = 0
    for x in range(x1, x2):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points

class graphNode(object):
    def __init__(self, node, previousNode, tree = None, grid = None, start = None, end = None):
        #node is a Node object, previousNode is a graphNode object
        #start and end are the NODES, not coordinates
        self.pathLength = 0
        self.node = node
        self.path = []
        self.neighbours = []
        self.count = 1

        if previousNode:
            self.previousNode = previousNode
            self.path = previousNode.path[:]
            self.path.append(node)
            self.pathLength = previousNode.pathLength
            self.pathLength += getDist(node,previousNode.node)
            self.start = previousNode.start
            self.end = previousNode.end
            self.tree = previousNode.tree
            self.grid = previousNode.grid
            self.count = previousNode.count + 1
        else:
            self.previousNode = None
            self.path.append(node)
            self.start = start
            self.end = end
            self.tree = tree
            self.grid = grid

        self.dist = getDist(self.node, self.end)

class aStar(object):
    def __init__(self, tree, start, end, map):
        startNode = tree.getPointNode(start)
        self.start = start
        self.end = end
        self.endNode = tree.getPointNode(end)##note that endnode is Node type while startNode is graphNode type
        self.startNode = graphNode(startNode, None, tree, map, startNode, self.endNode)
        self.tree = tree
        self.path = []
        self.visitedQueue = []
        self.priorityQueue = PriorityQueue()
        self.map = map[:]

    def Solve(self):
        count = 0
        self.priorityQueue.put((0,self.startNode))
        max = 0
        # for i in range(8):
        while (not self.path and self.priorityQueue.qsize()):
            if self.priorityQueue.qsize()>max:
                max = self.priorityQueue.qsize()
            bestNeighbour = self.priorityQueue.get()[1]
            bestNeighbour.node.getNeighbours(self.tree.root)
            self.visitedQueue.append(bestNeighbour.node)
            for neighbour in bestNeighbour.node.neighbours:
                # self.map[neighbour.center[1],neighbour.center[0]]=0
                if neighbour not in self.visitedQueue:
                    count +=1
                    neighbourGraphNode = graphNode(neighbour,bestNeighbour)
                    if not neighbourGraphNode.dist:
                        self.path = neighbourGraphNode.path
                        break
                    self.priorityQueue.put((1*neighbourGraphNode.dist+1*neighbourGraphNode.pathLength + 10*neighbourGraphNode.count,neighbourGraphNode))
                    self.visitedQueue.append(neighbourGraphNode.node)
        if not self.path:
            print "Impossible solution"
        print 'max queue size',max
        # return self.path

    def getPathCenterPoints(self):
        pathCenters = []
        pathCenters.append(self.start)
        for i in range(1,len(self.path)-2):
            pathCenters.append(self.path[i].center)
        pathCenters.append(self.end)
        return pathCenters

def drawPath(path,img=None):
    pathPixels = []
    for i in range(len(path) - 1):
        points = get_line(path[i], path[i + 1])
        pathPixels=pathPixels+points
    #if img != None:
    for i in range(len(path)-1):
        cv2.line(img,path[i],path[i+1],0,1)
    return pathPixels
