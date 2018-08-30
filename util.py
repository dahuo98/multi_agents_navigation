#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 13:17:39 2018

@author: dahuo
"""

import numpy as np
import math
import random

x=0
y=1

#position is the position of other agent
#radius is the sum of self.radius+other.radius
#with some error
def decideForward(forward):
    if random.uniform(0,1000)<forward*1000:
        return True
    else:
        return False
    
def findSlopesOfVOBoundry(position, combinedRadius):
    
    h=position[x]
    k=position[y]
    
    n=pow(h,2)+pow(k,2)-pow(combinedRadius,2)
    
    a=4*pow(k,2)-4*n
    b=8*h*k
    c=4*pow(h,2)-4*n
    
    if(pow(b,2)-4*a*c)<0:
        print 'b^2-4ac<0'
        return None,None
    
    
    
    
    root1=(1.0*(-b)+np.sqrt(pow(b,2)-4*a*c))/(2*a)
    root2=(1.0*(-b)-np.sqrt(pow(b,2)-4*a*c))/(2*a)
    
    slopes=[root1,root2]
    
    #find points of intersection
    z1=root1
    intersectA=1+pow(z1,2)
    intersectB=-2.0*h-2*z1*k
    intersectC=pow(h,2)+pow(k,2)-pow(combinedRadius,2)
    
    #print 'b^2-4ac= ',pow(intersectB,2)-4*intersectA*intersectC
    
    #x1=(-1.0*intersectB+np.sqrt(pow(intersectB,2)-4*intersectA*intersectC))/(2.0*intersectA)
    x1=(-1.0*intersectB+np.sqrt(0))/(2.0*intersectA)
    y1=x1*root1
    
    z2=root2
    intersectA=1+pow(z2,2)
    intersectB=-2*h-2*z2*k
    intersectC=pow(h,2)+pow(k,2)-pow(combinedRadius,2)
    
    #x2=(-1.0*intersectB+np.sqrt(pow(intersectB,2)-4*intersectA*intersectC))/(2.0*intersectA)
    x2=(-1.0*intersectB+np.sqrt(0))/(2.0*intersectA)
    y2=x2*root2
    
    intersectPoints=[(x1,y1),(x2,y2)]
    
    
    
    
    return slopes, intersectPoints



def isValidTriangle(pointA, pointB, pointC):
    a=np.linalg.norm(np.array(pointB)-np.array(pointC))
    b=np.linalg.norm(np.array(pointA)-np.array(pointC))
    c=np.linalg.norm(np.array(pointA)-np.array(pointB))
    
    if(a+b<=c):
        return False
    if(a+c<=b):
        return False
    if(b+c<=a):
        return False
    return True

#compute angle B of triangle ABC with sides a,b,c
#a is the oppsite side of angle A, b is oppsite side of angle B, c is oppisite side of angle C
#take in 3 points (x,y) tuples as arguments
#return angle in degrees
def getAngle(pointA, pointB, pointC):
    
    a=np.linalg.norm(np.array(pointB)-np.array(pointC))
    b=np.linalg.norm(np.array(pointA)-np.array(pointC))
    c=np.linalg.norm(np.array(pointA)-np.array(pointB))

    
    cosB=1.0*(a**2 + c**2 - b**2)/(2*a*c)

    
    return math.degrees(math.acos(cosB))


#find the nearest point from targetPoint to line(pointA, pointB)
#return coordinate of point(x,y), and dist from target point to line
def getNearestPointOnALine(pointA, pointB, targetPoint):
    x=0
    y=1
    newSlope=None
    if(pointA[x]-pointB[x]==0):       #the Line is vertical
        #new line is horizontal
        resultY=targetPoint[y]
        resultX=pointA[x]
        dist=np.linalg.norm(np.array(targetPoint)-np.array([resultX,resultY]))
        return (resultX,resultY), dist
        
        
    
    slope=1.0*(pointA[y]-pointB[y])/(pointA[x]-pointB[x])
        #if original line is horizontal
    if(slope==0):    
        #newSlope is undefined in this case, vertical line
        #x does not change
        resultX=targetPoint[x]
        resultY=pointA[y]
        dist=np.linalg.norm(np.array(targetPoint)-np.array([resultX,resultY]))
        return (resultX,resultY), dist
        
        
        
    newSlope=-1/slope
        
    attr=pointA[y]-slope*pointA[x]
   
    
    
    
    
    newAttr=targetPoint[y]-newSlope*targetPoint[x]
    
    resultX=1.0*(newAttr-attr)/(slope-newSlope)
    resultY=newSlope*resultX+newAttr
    
    dist=np.linalg.norm(np.array(targetPoint)-np.array([resultX,resultY]))
    
    return (resultX,resultY), dist


#the orthogonal projection of vector u onto vector v
def getOrthogoalProjection(u, v):
    
    distSq=np.dot(v,v)
    if distSq ==0:
        return 0
    
    projection=np.dot(np.divide(np.dot(u,v),distSq),v)
    
    #if the projection is longer than v, return full vector v
    if np.linalg.norm(projection)>np.linalg.norm(v):
        return v
    else:
        return projection
    
    
    
    
    
    
def checkOrientation(p1,p2,p3):
    
    # compare slope for line p1p2 and slope for p2p3
    #if s is 0, slope of p1p2=slope of p2p3, 
    #if s is >0, slope of p1p2>slope of p2p3, 
    #if s is <0, slope of p1p2<slope of p2p3, 
    
    s=(p2[1]-p1[1])*(p3[0]-p2[0])-(p3[1]-p2[1])*(p2[0]-p1[0])
    
    if(s==0):
        return 0    #colinear
    elif(s<0):
        return -1   #counterclockwise
    else:
        return 1    #clockwise

#this function checks if point target is on line p1p2    
def isOnLineSegment(p1,p2,target):
    
    if(target[0]<=max(p1[0],p2[0]) and target[0]>=min(p1[0],p2[0]) and target[1]<=max(p1[1],p2[1]) and target[1]>=min(p1[1],p2[1])):
        return True
    return False
    

#this function checks if line p1p2 intersets with line t1t2
def isIntersect(p1,p2,t1,t2):
    if set(p1)==set(t1) or set(p1)==set(t2) or set(p2)==set(t1) or set(p2)==set(t2):
        return False
    
    #get the orientations of p1p2t1,p1p2t2,t1t2p1,t1t2p2
    o1=checkOrientation(p1,p2,t1)
    o2=checkOrientation(p1,p2,t2)
    o3=checkOrientation(t1,t2,p1)
    o4=checkOrientation(t1,t2,p2)
    
    #if o1!=o2 && o3!=o4, then line p1p2,t1t2 must intersect with each other
    if(o1!=o2 and o3!=o4):
        return True
    
    #the case of part of t1t2 is on p1p2, in this case we consider this as not intersect 
    #since two points on the same side of polygon should be visible to each other
    if(o1==o2 and o2==o3 and o3==o4 and o4==0):
        return False
    
    
    
    #check for special cases, that is when o1 or o2 or o3 or o4 is colinear and the point is on the line segment 
    if(o1==0 and isOnLineSegment(p1,p2,t1)):
        return True
    if(o2==0 and isOnLineSegment(p1,p2,t2)):
        return True
    if(o3==0 and isOnLineSegment(t1,t2,p1)):
        return True
    if(o4==0 and isOnLineSegment(t1,t2,p2)):
        return True
    
    return False




#fit power1 polynomial
#p1,p2 has the following structure [x,y]
def fitPolynomial(p1, p2):
    xdiff=p1[0]-p2[0]
    ydiff=p1[1]-p2[1]
    
    
    #if the line is a vertical line, has form of x=constant, the constant will be returned as attr
    if(xdiff==0):
        slope=None
        attr=p1[0]
        return slope, attr
    
    slope=ydiff*1.0/xdiff
    attr=p1[1]-slope*p1[0]
    
    #the equation should be y= slope*x + attr
    return slope, attr



#converts slopes of a linear equation line to the Acute angle in radian with respect to x axis
def slopeToRadian(slope):
    
    x1=0
    x2=1
    
    y1=x1*slope
    y2=x2*slope
    
    return math.atan((abs(y1-y2)/abs(x1-x2)))

#returrn angle in degrees
def getAngleBetweenVectorAndPositiveXAxis(vector):
    
    if vector[0]==0 and vector[1]==0:
        return None
    
    if vector[1]>=0:
        #vector above x-axis
        return math.degrees(math.acos(vector[0]/np.linalg.norm(vector)))
    else:
        return 360.0-math.degrees(math.acos(vector[0]/np.linalg.norm(vector)))
    
    
def getIntersectionPointOfRays(origin1, direction1, origin2, direction2):
    
    slope1, attr1=fitPolynomial(origin1, direction1)
    slope2, attr2=fitPolynomial(origin2, direction2)
    
    #print slope1, slope2
    
    if slope1 == slope2:
        return None
    
    if slope1 is None:
        #ray1 is verticle 
        point= (origin1[0],slope2*origin1[0]+attr2)
        if isOnLineSegment(origin1,np.dot(np.subtract(direction1,origin1),100000000),point) and isOnLineSegment(origin2,np.dot(np.subtract(direction2,origin2),100000000),point):
            return point
        else:
            return None
        
    
    if slope2 is None:
        #ray2 is verticle 
        point= (origin2[0],slope1*origin2[0]+attr1)
        
        if isOnLineSegment(origin1,np.dot(np.subtract(direction1,origin1),100000000),point) and isOnLineSegment(origin2,np.dot(np.subtract(direction2,origin2),100000000),point):
            return point
        else:
            return None
    
    
    
    x=(attr2-attr1)/(slope1-slope2)
    point= (x,slope1*x+attr1)
    #print point
    if isOnLineSegment(origin1,np.dot(np.subtract(direction1,origin1),100000000),point) and isOnLineSegment(origin2,np.dot(np.subtract(direction2,origin2),100000000),point):
        return point
    else:
        return None
    
    
    


if __name__=='__main__':
    
    
    #print getIntersectionPointOfRays((0,0), (1,0), (1,0), (0.5,0.5))
    #print isOnLineSegment((0,0),(5,5),(3,3.5))
    
    apex= [ 0.1161565 ,  0.37968057]
    leftLine= [ 0.11613275 , 0.37970882]
    rightLine= [ 0.11618025 , 0.37965232]
    apex1= [ 0.02227795 , 0.19381791]
    leftLine1=[-0.0267927   ,0.03231972]
    rightLine1=[ 0.09772082  ,0.34480786]
    
    
    #print getIntersectionPointOfRays(apex,leftLine,apex1,leftLine1)
    print getIntersectionPointOfRays(apex,leftLine,apex1,rightLine1)
    #print getIntersectionPointOfRays(apex,rightLine,apex1,leftLine1)
    #print getIntersectionPointOfRays(apex,rightLine,apex1,rightLine1)
        
                         
                     
        
    
    
    
    
    
    
    