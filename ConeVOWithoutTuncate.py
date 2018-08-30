#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 29 12:28:06 2018

@author: dahuo
"""
import util
import numpy as np

class VO:
    
    
    #left line is a point on the vector line of the lefter boundry, with origin to be the apex   
    #right line is a point on the vector line of the righter boundry, with origin to be the apex
    def __init__(self, apex, leftLine, rightLine):
        self.apex=apex
        self.leftLine=leftLine
        self.rightLine=rightLine
        
    
    '''
    #left slope is the slope of the lefter boundry, with origin to be the apex   
    #right slope is the slope of the righter boundry, with origin to be the apex  
    def __init__(self, apex, leftSlope, rightSlope):
        self.apex=apex
        self.leftSlope=leftSlope
        self.rightSlope=rightSlope
    '''    
    
    
    def isPointInVO(self,point):
        
        ERROR_THREHOLD=0.1
        #print 'point is: ',point
        
        relativeLeftLine=np.subtract(self.leftLine,self.apex)
        relativeRightLine=np.subtract(self.rightLine,self.apex)
        relativePoint=np.subtract(point, self.apex)
        if relativePoint[0]==0 and relativePoint[1]==0:
            return False
        
        leftAngle=util.getAngleBetweenVectorAndPositiveXAxis(relativeLeftLine)
        rightAngle=util.getAngleBetweenVectorAndPositiveXAxis(relativeRightLine)
        pointAngle=util.getAngleBetweenVectorAndPositiveXAxis(relativePoint)
        
        if abs(leftAngle-rightAngle)>180:
            #more than one round, say a=35, b=275
            #has to add 360 to smaller angle
            if leftAngle<rightAngle:
                leftAngle+=360
            else:
                rightAngle+=360
                
            #add 360 to pointAngle as well
            if pointAngle<180:
                pointAngle+=360
        
        #print 'leftAngle: ',leftAngle
        #print 'rightAngle: ',rightAngle
        #print 'pointAngle: ', pointAngle
        if leftAngle<rightAngle:
            #return pointAngle>=leftAngle and pointAngle<=rightAngle
            #print 'pointAngle>leftAngle+ERROR_THREHOLD and pointAngle<rightAngle-ERROR_THREHOLD',
            #print pointAngle>leftAngle+ERROR_THREHOLD and pointAngle<rightAngle-ERROR_THREHOLD
            return pointAngle>leftAngle+ERROR_THREHOLD and pointAngle<rightAngle-ERROR_THREHOLD
        else:
            #return pointAngle>=rightAngle and pointAngle<=leftAngle
            return pointAngle>rightAngle+ERROR_THREHOLD and pointAngle<leftAngle-ERROR_THREHOLD
        
    def getIntersectionPoints(self,vo):
        
        point1=util.getIntersectionPointOfRays(self.apex,self.leftLine, vo.apex, vo.leftLine)
        point2=util.getIntersectionPointOfRays(self.apex,self.leftLine, vo.apex, vo.rightLine)
        point3=util.getIntersectionPointOfRays(self.apex,self.rightLine, vo.apex, vo.leftLine)
        point4=util.getIntersectionPointOfRays(self.apex,self.rightLine, vo.apex, vo.rightLine)
        
        rv=[]
        
        if point1 is not None:
            rv.append(point1)
        if point2 is not None:
            rv.append(point2)
        if point3 is not None:
            rv.append(point3)
        if point4 is not None:
            rv.append(point4)
            
        return rv
    
if __name__ == '__main__':
    vo=VO((-0.0159517 ,  0.12363075),(-0.09780194 , 0.00715866),( 0.08157027 , 0.22733549))
    print vo.isPointInVO(((0.28792879870952603, -0.23259442876394337)))    