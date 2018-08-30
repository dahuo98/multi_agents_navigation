#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 28 23:03:33 2018

@author: dahuo
"""

import numpy as np
from ConeVOWithoutTuncate import VO
from util import findSlopesOfVOBoundry
from math import atan2
import math
import util
import heapq
#import sys
import random

x=0
y=1

MOVE_BACK=10086
AVOID_VELOCITY=[0,0]

class Agent:
    
    def __init__(self, id,radius, maxSpeed,position,velocity,prefSpeed,goalPoint,timeWindow=None, agentNeighbors=None, newVelocity=None,VOs=None, simulator=None):
        self.id=id
        self.radius=radius
        self.maxSpeed=maxSpeed
        self.position=position
        self.velocity=velocity
        self.prefVelocity=None
        self.timeWindow=timeWindow
        self.agentNeighbors=agentNeighbors
        self.newVelocity=newVelocity
        self.VOs=VOs
        self.sim=simulator
        self.goalPoint=goalPoint
        self.prefSpeed=prefSpeed
        self.updatePrefVelocity()
        
        
        
    def computeNeighbors(self):
        
        self.agentNeighbors=[]
        self.agentNeighbors=self.sim.computeAgentNeighbors(self.id);
        return
    
    
    #all velocityObstacles are with respect to current agent, not global coordinate
    def setVelocityObstacles(self):
        
        #ERROR_THRESHOLD=0.1 
       # print self.agentNeighbors
        if self.agentNeighbors==None or self.agentNeighbors==[]:
            self.VOs=None
            return
        
        self.VOs=[]
        
        for agent in self.agentNeighbors:
            
            combinedRadius=self.radius+agent.radius
            
            #relative position of agent with respect to self
            relativePosition=np.subtract(agent.position, self.position)
            
            slopes, points=findSlopesOfVOBoundry(relativePosition, combinedRadius)
            
            if slopes is None and points is None:
                self.VOs=MOVE_BACK
                relative_pos=np.subtract(agent.position,self.position)
                dist=combinedRadius-np.linalg.norm(relative_pos)*1.1
                AVOID_VELOCITY=np.dot(relative_pos,dist/np.linalg.norm(relative_pos)*-1)
                return
            
            #print 'agent: ',agent.position, 'slopes: ', slopes
            
            #determine which one is the lefter line
            
            #point1=(relativePosition[x],slopes[0]*relativePosition[x])
            #point2=(relativePosition[x],slopes[1]*relativePosition[x])
            point1=points[0]
            point2=points[1]
            vo=None
            if(math.degrees(atan2(np.linalg.det([point1,point2]),np.dot(point1,point2)))>0):
                #point1 is on lefter line
                #vo=VO(self.position, slopes[0], slopes[1])
                #vo=VO(self.position, point1, point2)
                vo=VO([0,0], point1, point2)
                
                #vo=VO(self.position, )
                
            else:
                #vo=VO(self.position, point2, point1)
                vo=VO([0,0], point2, point1)
                
                
            #move VO to (VA+VB)/2
            
            
            
            vo.apex=np.add(vo.apex, np.divide(np.add(self.velocity, agent.velocity),2.0))
            vo.leftLine=np.add(vo.leftLine,np.divide(np.add(self.velocity, agent.velocity),2.0) )
            vo.rightLine=np.add(vo.rightLine, np.divide(np.add(self.velocity, agent.velocity),2.0) )
            
            #vo.apex=np.add(vo.apex, np.divide(np.subtract( agent.velocity,self.velocity),2.0))
            #vo.leftLine=np.add(vo.apex, np.divide(np.subtract( agent.velocity,self.velocity),2.0))
            #vo.rightLine=np.add(vo.apex, np.divide(np.subtract( agent.velocity,self.velocity),2.0))
            
            #print 'avg velocity: ',np.divide(np.add(self.velocity, agent.velocity),2.0)
            #print self.position,
            #print ':'
            #print 'apex: ',vo.apex
            #print 'left: ',vo.leftLine
            #print 'right: ',vo.rightLine
            
            self.VOs.append(vo)
            
    
    def computeVelocityFor2(self):
        
        if self.VOs is None:
            #no need to change velocity
            self.newVelocity=self.prefVelocity
            return
        
        #if prefVelocity is outside of VOS, do nothing, newV=prefV
        vo=self.VOs[0]
        if not vo.isPointInVO(self.prefVelocity):
            #print 'not in VO'
            self.newVelocity=self.prefVelocity
            return
        
        
       
        vLeft=[]
        vRight=[]
        leftDist=None
        rightDist=None
        
        for vo in self.VOs:
            
            #process left line, find closest point on left line
            if not util.isValidTriangle(self.prefVelocity, vo.apex, vo.leftLine):
                #self.location is on the same line of left line
                #print 'not valid triangle L'
                
                dist1=np.linalg.norm(np.subtract(self.prefVelocity,vo.apex))
                dist2=np.linalg.norm(np.subtract(self.prefVelocity,vo.leftLine))
                
                if(dist1<dist2):
                    #apex is cloest point
                    vLeft=vo.apex
                    leftDist=dist1
                else:
                    #prefVelocity on left line, cloest point is prefVelocity itself
                    vLeft=self.prefVelocity
                    leftDist=0
               
            else:
                #print 'valid triangle L'
                angle=util.getAngle(self.prefVelocity, vo.apex, vo.leftLine)
               # print 'L ',angle
                #closest point is apex
                if(angle>=90):
                    vLeft=vo.apex
                    leftDist=np.linalg.norm(vo.apex)
                else:
                    point, leftDist=util.getNearestPointOnALine(vo.apex, vo.leftLine, self.prefVelocity)
                    vLeft=point
                        
                        
            #process right line, find closest point on right line
            if not util.isValidTriangle(self.prefVelocity, vo.apex, vo.rightLine):
                #self.location is on the same line of left line
                #print 'not valid triangle R'
                
                dist1=np.linalg.norm(np.subtract(self.prefVelocity,vo.apex))
                dist2=np.linalg.norm(np.subtract(self.prefVelocity,vo.rightLine))
                
                if(dist1<dist2):
                    #apex is cloest point
                    vRight=vo.apex
                    rightDist=dist1
                else:
                    #prefVelocity on left line, cloest point is prefVelocity itself
                    vRight=self.prefVelocity
                    rightDist=0
               
            else:
                #print 'valid triangle R'
                
                angle=util.getAngle(self.prefVelocity, vo.apex, vo.rightLine)
                #print 'R ',angle
                #closest point is apex
                if(angle>=90):
                    
                    vRight=vo.apex
                    rightDist=np.linalg.norm(vo.apex)
                else:
                    point, rightDist=util.getNearestPointOnALine(vo.apex, vo.rightLine, self.prefVelocity)
                    vRight=point
                    
            
            leftIntersect=util.isIntersect(vo.apex,vo.rightLine,(0,0),vLeft)
            rightIntersect=util.isIntersect(vo.apex,vo.leftLine,(0,0),vRight)
            
            if(not leftIntersect and not rightIntersect):
                if leftDist < rightDist:
                    self.newVelocity=vLeft
                else:
                    self.newVelocity=vRight
            elif leftIntersect:
                self.newVelocity=vRight
            else:
                self.newVelocity=vLeft
            
                        
                        
                        
        
        #project prefered velocity to line(self.position, minPoint)
        #print 'newVelocity:',
        #print self.newVelocity

        
        return
    def getRandomMove(self):
        s=[0,0]
        for other in self.agentNeighbors:
            s=np.add(s,np.dot(-1,np.divide(np.subtract(other.position,self.position),np.linalg.norm(np.subtract(other.position,self.position)))))
        s=np.add(s,np.dot(1,self.prefVelocity))
        
        return np.divide(s,np.linalg.norm(s))
            
    
    
    def computeVelocity(self):
        
        CHANCE=0.01
        
        if self.VOs is None:
            
            #no need to change velocity
            self.newVelocity=self.prefVelocity
            return
        
        if self.VOs == MOVE_BACK:
            self.newVelocity=AVOID_VELOCITY
            return
            if util.decideForward(CHANCE):
                #self.newVelocity=np.dot(random.uniform(0,1),self.newVelocity)
                self.newVelocity=self.getRandomMove()
                #self.newVelocity=np.add(np.dot(0.5,self.prefVelocity),np.dot(0.5,[random.random(),random.random()]))
            else:
                #self.newVelocity=np.dot(random.uniform(-1,0),self.newVelocity)
                #self.newVelocity=np.add(np.dot(-0.5,self.prefVelocity),np.dot(-0.5,[random.random(),random.random()]))
                self.newVelocity=[0,0]
            return
        
        #if prefVelocity is outside of VOS, do nothing, newV=prefV
        #test if prefVelocity is inside any of the VOs
        inVOs=False
        for vo in self.VOs:
            if  vo.isPointInVO(self.prefVelocity):
                inVOs=True
                break
                #print 'not in VO'
        if not inVOs:
            
            self.newVelocity=self.prefVelocity
            return
        
        
        
        vLeft=[]
        vRight=[]
        leftDist=None
        rightDist=None
        
        #points inside has the structure: (dist with prefV, (x,y))
        closestPoints=[]
        
        for vo in self.VOs:
            
            #process left line, find closest point on left line
            if not util.isValidTriangle(self.prefVelocity, vo.apex, vo.leftLine):
                #self.location is on the same line of left line
                #print 'not valid triangle L'
                
                dist1=np.linalg.norm(np.subtract(self.prefVelocity,vo.apex))
                dist2=np.linalg.norm(np.subtract(self.prefVelocity,vo.leftLine))
                
                if(dist1<dist2):
                    #apex is cloest point
                    vLeft=vo.apex
                    leftDist=dist1
                else:
                    #prefVelocity on left line, cloest point is prefVelocity itself
                    vLeft=self.prefVelocity
                    leftDist=0
               
            else:
                #print 'valid triangle L'
                angle=util.getAngle(self.prefVelocity, vo.apex, vo.leftLine)
               # print 'L ',angle
                #closest point is apex
                if(angle>=90):
                    vLeft=vo.apex
                    leftDist=np.linalg.norm(np.subtract(self.prefVelocity,vo.apex))
                else:
                    point, leftDist=util.getNearestPointOnALine(vo.apex, vo.leftLine, self.prefVelocity)
                    vLeft=point
                        
                        
            #process right line, find closest point on right line
            if not util.isValidTriangle(self.prefVelocity, vo.apex, vo.rightLine):
                #self.location is on the same line of left line
                #print 'not valid triangle R'
                
                dist1=np.linalg.norm(np.subtract(self.prefVelocity,vo.apex))
                dist2=np.linalg.norm(np.subtract(self.prefVelocity,vo.rightLine))
                
                if(dist1<dist2):
                    #apex is cloest point
                    vRight=vo.apex
                    rightDist=dist1
                else:
                    #prefVelocity on left line, cloest point is prefVelocity itself
                    vRight=self.prefVelocity
                    rightDist=0
               
            else:
                #print 'valid triangle R'
                
                angle=util.getAngle(self.prefVelocity, vo.apex, vo.rightLine)
                #print 'R ',angle
                #closest point is apex
                if(angle>=90):
                    
                    vRight=vo.apex
                    rightDist=np.linalg.norm(np.subtract(self.prefVelocity,vo.apex))
                else:
                    point, rightDist=util.getNearestPointOnALine(vo.apex, vo.rightLine, self.prefVelocity)
                    vRight=point
                    
            #check if new velocity is crossing with the boundries
            leftIntersect=util.isIntersect(vo.apex,vo.rightLine,(0,0),vLeft)
            rightIntersect=util.isIntersect(vo.apex,vo.leftLine,(0,0),vRight)
            #print 'vleft: '+str(vLeft)
            #print 'vRight: '+str(vRight)
            #print leftIntersect
            #print rightIntersect
            leftIntersect=False
            rightIntersect=False
            
            
            if(not leftIntersect and not rightIntersect):
                heapq.heappush(closestPoints,(leftDist,np.array(vLeft).tolist()))
                heapq.heappush(closestPoints,(rightDist,np.array(vRight).tolist()))
                #if leftDist < rightDist:
                #    self.newVelocity=vLeft
                #else:
                #    self.newVelocity=vRight
            elif leftIntersect:
                #self.newVelocity=vRight
                heapq.heappush(closestPoints,(rightDist,np.array(vRight).tolist()))
            else:
                heapq.heappush(closestPoints,(leftDist,np.array(vLeft).tolist()))
                #self.newVelocity=vLeft
        
        #for every points in cloestPoints, check if these points are inside any VOs
        #if true, then pass current point, move to next cloestest points
        #while list is not empty
        while(closestPoints):
            
            curr=heapq.heappop(closestPoints)
            #print 'curr is: '+str(curr)
            
            inVO=False
            
            for vo in self.VOs:
                if  vo.isPointInVO(curr[1]):
                    inVO=True
                    #print 'inVO=True'
                    break
            #inVO=False    
            if not inVO:
                
                self.newVelocity=curr[1]
                return
            
        #return None
        #self.newVelocity=[0,0]
        #return
        
        #if all points are in VO:
        #find intersection points between VOs
        intersections=[]
        for vo in self.VOs:
            for other in self.VOs:
                if vo is other:
                    continue
                intersections+=vo.getIntersectionPoints(other)
        
        points=[]
        for pt in intersections:
            heapq.heappush(points,(np.linalg.norm(np.subtract(pt,self.prefVelocity)),pt))
        #print 'points is:',
        #print points
        while(points):
            
            curr=heapq.heappop(points)
            #print 'curr intersection is: '+str(curr)
            
            inVO=False
            
            for vo in self.VOs:
                if  vo.isPointInVO(curr[1]):
                    inVO=True
                    break
            #inVO=False    
            if not inVO:
                self.newVelocity=curr[1]
                return
                '''
                #apply random number to this dangerous point
                print 'use random point'
                
                while(True):
                    newPoint=(curr[1][0]+random.uniform(-0,0),curr[1][1]+random.uniform(-0,0))
                    newPointInVO=False
                    for vo in self.VOs:
                        
                        if vo.isPointInVO(newPoint):
                            newPointInVO=True
                            break
                    if not newPointInVO:
                        self.newVelocity=newPoint
                        return
                            
                
                
                #self.newVelocity=curr[1]
                #return
                '''
        print 'applying random'
        if util.decideForward(0.3):
            #self.newVelocity=np.dot(random.uniform(0,1),self.newVelocity)
            self.newVelocity=self.getRandomMove()
        else:
            #self.newVelocity=np.dot(random.uniform(-1,0),self.newVelocity)
            self.newVelocity=[0,0]
        #print 'no good new velocity, move back 1 step'
        return
    
        
                
    
            
            
            
        
        
        
            
        
        
        
            
            
  
    
    def updateNewVelocity(self):
        #print 'agent ',self.id,' location: ',self.getAgentPosition()
        self.computeNeighbors()
        self.setVelocityObstacles()

        if np.linalg.norm(np.subtract(self.position,self.goalPoint))<0.01 and self.VOs is None:
            
            self.newVelocity=[0,0]
            return
        
        #print self.getStrVOs()
        #print 'prefVelocity is: '+str(self.prefVelocity)
        self.computeVelocity()
        #print 'all: ',self.newVelocity
        #self.computeVelocityFor2()
        #print 'for 2:', self.newVelocity
        #self.computeVelocity()
        #print 'agent '+str(self.id)+'new Velocity: '+str(self.newVelocity)
        #print ''
        
        
        return
    
    
    
    
    def updateVelocity(self):
        self.velocity=self.newVelocity
        return
    
    def updatePosition(self,timeStep):
        
        
        
        self.position=np.add(self.position, np.dot(self.velocity,timeStep))
        self.updatePrefVelocity()
        
        return
    
    def getAgentPosition(self):
        return self.position
    
    def getStrVOs(self):
        
        if self.VOs is None:
            return 'None'
        
        if self.VOs == MOVE_BACK:
            return 'MOVE BACK 1 STEP'
        
        rv='agent '+str(self.id)+' VOs: \n'
        for vo in self.VOs:
            rv+='apex: '+str(vo.apex)+'\n'
            rv+='leftLine: '+str(vo.leftLine)+'\n'
            rv+='rightLine: '+str(vo.rightLine)#+'\n'
        return rv
    
    #assume given a goal point that is not inf
    def updatePrefVelocity(self):
        
        if np.linalg.norm(np.subtract(self.goalPoint,self.position))>self.prefSpeed:
        
            slope, attr=util.fitPolynomial(self.position, self.goalPoint)
            
            if slope is None:
                if self.position[y]-self.goalPoint[y] >0:
                    #prefV point downward
                    self.prefVelocity=[0,-1.0*self.prefSpeed]
                else:
                     self.prefVelocity=[0,1.0*self.prefSpeed]
                return
            
            angle=util.slopeToRadian(slope)
            xdiff=None
            #print 'angle is', math.degrees(angle)
            
            if self.position[x]-self.goalPoint[x] >0:
                #goal is on the left hand side of current location
                xdiff=math.cos(angle)*(-1.0*self.prefSpeed)
            else:
                #goal is on the right hand side of current location
                xdiff=math.cos(angle)*(1.0*self.prefSpeed)
                
            ydiff=xdiff*slope
            
            self.prefVelocity= [xdiff,ydiff]
        else:
            self.prefVelocity=np.subtract(self.goalPoint,self.position)
        
        
    
    
'''            
    def computeVelocity(self):
        
        if self.VOs is None:
            #no need to change velocity
            self.newVelocity=self.prefVelocity
            return
        
       
        #distance and line from self.location to closest point on boundry
        minDist=sys.maxint
        minPoint=(0,0)
        #[start pt, end pt]
        #minLine=[(0,0),(0,0)]
        
        for vo in self.VOs:
            
            #process left line, find closest point on left line
            if not util.isValidTriangle(self.position, vo.apex, vo.leftLine):
                #self.location is on the same line of left line
                
                #assume self.location cannot be on the boundry of vo
                dist=np.linalg.norm(np.subtract(self.position,vo.apex))
                if(dist<minDist):
                    minDist=dist
                    minPoint=vo.apex
            else:
                
                angle=util.getAngle(self.position, vo.apex, vo.leftLine)
                
                #closest point is apex
                if(angle>=90):
                    dist=np.linalg.norm(np.subtract(self.position,vo.apex))
                    if dist<minDist:
                        minDist=dist
                        minPoint=vo.apex
                else:
                    point, dist=util.getNearestPointOnALine(vo.apex, vo.leftLine, self.position)
                    if dist< minDist:
                        minDist=dist
                        minPoint=point
                        
                        
            #process right line, find closest point on left line
            if not util.isValidTriangle(self.position, vo.apex, vo.rightLine):
                #self.location is on the same line of left line
                
                #assume self.location cannot be on the boundry of vo
                dist=np.linalg.norm(np.subtract(self.position,vo.apex))
                if(dist<minDist):
                    minDist=dist
                    minPoint=vo.apex
            else:
                
                angle=util.getAngle(self.position, vo.apex, vo.rightLine)
                
                #closest point is apex
                if(angle>=90):
                    dist=np.linalg.norm(np.subtract(self.position,vo.apex))
                    if dist<minDist:
                        minDist=dist
                        minPoint=vo.apex
                else:
                    point, dist=util.getNearestPointOnALine(vo.apex, vo.rightLine, self.position)
                    if dist< minDist:
                        minDist=dist
                        minPoint=point
                        
                        
                        
        
        #project prefered velocity to line(self.position, minPoint)

        minLineVector=np.subtract(minPoint, self.position)
        print 'minLineVector is: ', minLineVector
        self.newVelocity=util.getOrthogoalProjection(self.prefVelocity, minLineVector)
        print 'new velocity is: ',self.newVelocity
        return
'''  
    
            
                    
                    
                    
        
'''   
    def setVelocityObstacles(self):
        
       # print self.agentNeighbors
        if self.agentNeighbors==None or self.agentNeighbors==[]:
            self.VOs=None
            return
        
        self.VOs=[]
        
        for agent in self.agentNeighbors:
            
            combinedRadius=self.radius+agent.radius
            
            #relative position of agent with respect to self
            relativePosition=np.subtract(agent.position, self.position)
            
            slopes=findSlopesOfVOBoundry(relativePosition, combinedRadius)
            
            #determine which one is the lefter line
            
            point1=(1,slopes[0])
            point2=(1,slopes[1])
            vo=None
            if(math.degrees(atan2(np.linalg.det([point1,point2]),np.dot(point1,point2)))>0):
                #point1 is on lefter line
                vo=VO(self.position, slopes[0], slopes[1])
                
                #vo=VO(self.position, )
                
            else:
                vo=VO(self.position, slopes[1], slopes[0])
                
                
            #move VO to (VA+VB)/2
            
            
            
            vo.apex=np.add(vo.apex, np.divide(np.add(self.velocity, agent.velocity),2))
            
            self.VOs.append(vo)
'''  


          
            
if __name__=='__main__':           
                
                            
    #vo=VO([4.17,2.11],[1.82,5.93], [1.82,-1.71])
    vo=VO([0.5,0.68265637],[-1.84762024 , 4.50191891], [-1.84762024, -3.13660616])
    agent=Agent(3,1, 1,[3.67381012,  1.43420438],[0.45158789771678459, 0.68265637321062411],[0,1])
    agent.VOs=[vo]
    agent.computeVelocityFor2()
    print agent.newVelocity
    





                					
				
                    
                        
        
    
    
                
                
                
            
            
            
            
            
            
            
            
            
        
    
    
    