#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  4 17:08:37 2018

@author: dahuo
"""

from Agent import Agent
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from random import random

MIN_TIMEWINDOWCOUNT=0.1
SAFE_MARGIN=0

class RVOSimulator:
    
    #timeStep is the timeStep of the simulation, must>0
    #timeWindow is the minimal amount of time for which a new agent's velocities that are computed by the simulation are safe with respect to other agents.
    #maxNeighborDist is the maximum distance (center point to center point) to other agents a new agent takes into account in the navigation.
    #default of maxNeighborDist is (Va+Vb)*t
    
    def __init__(self,timeStep=0.05, timeWindow=0.2, maxNeighborDist=None):
        self.agentList=[]
        self.timeStep=timeStep
        self.timeWindow=timeWindow
        self.maxNeighborDist=maxNeighborDist
        self.timeWindowCount=0
        self.globalTime=0.0
        
    def addAgent(self,position, radius, maxSpeed, velocity,prefSpeed,goalPoint):
        #x=position[0]+random()*0.1
        #y=position[1]+random()*0.1
        #position=(x,y)
        #id counts from 0
        newId=len(self.agentList)
        agent=Agent(newId, radius,maxSpeed,position,velocity,prefSpeed,goalPoint,timeWindow=self.timeWindow, simulator=self)
        self.agentList.append(agent)
        return
    
    def computeAgentNeighbors(self, agentId):
        
        agent=self.agentList[agentId]
        neighbors=[]
        
       
        for other in self.agentList:
            if other.id==agentId:
                continue
            
            if self.maxNeighborDist is None:
                #use sum of max speed as max neighbor dist
                if np.linalg.norm(np.subtract(agent.position, other.position)) -agent.radius-other.radius > SAFE_MARGIN+(agent.maxSpeed+other.maxSpeed)*self.timeWindow:
                    #no collision within the time window
                    continue
                else:
                    #add this agent in neighbors
                    neighbors.append(other)
            else:
                #use maxNeighborDist
                if np.linalg.norm(np.subtract(agent.position, other.position))-agent.radius-other.radius > self.maxNeighborDist:
                    #no collision within the time window
                    continue
                else:
                    #add this agent in neighbors
                    neighbors.append(other)
                    
        
        return neighbors
    
    
    
    
    #do one time step assume we do not need to change velocity
    def doStep(self):
        #print 'timeWindowCount is:', self.timeWindowCount
        #if self.timeWindowCount<=0:
        if self.timeWindowCount<=MIN_TIMEWINDOWCOUNT:
            
            #re-compute velocity for all agents
            self.timeWindowCount=self.timeWindow
            for agent in self.agentList:
                agent.updateNewVelocity()
            
            #update velocity only if all agents are finished computing new velocity
            for agent in self.agentList:
                agent.updateVelocity()
        
        for agent in self.agentList:
            
            agent.updatePosition(self.timeStep)
            #print agent.getAgentPosition
        
        self.timeWindowCount -= self.timeStep
        self.globalTime += self.timeStep
        
        
    
            
            
        
        
                    
    
    def getNumberOfAgents(self):
        return len(self.agentLst)
    
    
    
if __name__== '__main__':
    #'''
    #numOfAgents=input('how many agents?')
    #plt.ion()
    
    
    sim=RVOSimulator()
    
    colors=[]
    
    start1=(0.1,0.1)
    start2=(0,5.1)
    start3=(3.1,-0.1)
    start4=(3,4.9)
    start5=(6.1,0.2)
    start6=(6,5.2)
    dest1=(0.1,5)
    dest2=(0,0)
    dest3=(3.1,5)
    dest4=(3,0)
    dest5=(6.1,5)
    dest6=(6,0)
    sim.addAgent(start1,1,1.1,[0,0],1,dest1)
    sim.addAgent(start2,1,1.1,[0,0],1,dest2)
    sim.addAgent(start3,1,1.1,[0,0],1,dest3)
    sim.addAgent(start4,1,1.1,[0,0],1,dest4)
    sim.addAgent(start5,1,1.1,[0,0],1,dest5)
    sim.addAgent(start6,1,1.1,[0,0],1,dest6)
    '''
    for i in range(numOfAgents):
        
        x=input('x coordinate of agent'+str(i+1)+': \n')
        y=input('y coordinate of agent'+str(i+1)+': \n')
        
        goalx=input('x coordinate of goal of agent'+str(i+1)+': \n')
        goaly=input('y coordinate of goal of agent'+str(i+1)+': \n')
        
        sim.addAgent((x,y),1,1.5,[0,0],1,(goalx,goaly))
        colors.append(matplotlib.colors.ListedColormap ( np.random.rand ( 256,3)))
    '''
    plt.axis([-2, 12, -2, 12])    
    for i in range(10000):
        
        #for agent in sim.agentList:
            #print 'agent ',agent.id,' location: ',agent.getAgentPosition()
            #print agent.getStrVOs()
            #print ''
            #print ''
        
            
        sim.doStep()
        
        arrive=False
        #check if arrive
        for agent in sim.agentList:
            if(np.linalg.norm(np.subtract(agent.goalPoint,agent.position))<0.001):
                arrive=True
            else:
                arrive=False
        
        if arrive:
            break
        
        #print ''
        #print ''
         #xs=[]
         #ys=[]
        for i in range(len(sim.agentList)):
             agent=sim.agentList[i]
                
             position=agent.getAgentPosition()
             #xs.append(position[0])
             #ys.append(position[1])
             
             plt.plot(position[0],position[1],'go')
             
             
             
        
        #plt.show(block=False)
        plt.draw()
        #plt.show()
        plt.pause(0.00001)
            #plt.close()
                
    '''    
        
        
        
        

    
    sim=RVOSimulator()
    #position, radius, maxSpeed, velocity,prefSpeed,goalPoint
    start1=(0,0)
    start2=(5,5)
    start3=(3,0)
    start4=(8,5)
    dest1=(5,5)
    dest2=(0,0)
    dest3=(8,5)
    dest4=(3,0)
    sim.addAgent(start1,1,1.1,[0,0],1,dest1)
    sim.addAgent(start2,1,1.1,[0,0],1,dest2)
    sim.addAgent(start3,1,1.1,[0,0],1,dest3)
    sim.addAgent(start4,1,1.1,[0,0],1,dest4)
    #sim.addAgent((0,0),1,1,[1,0.1],[1,0.1])
    #sim.addAgent((3,0),1,1,[-1,-0.1],[-1,-0.1])
    agentA=[]
    agentB=[]
    agentC=[]
    agentD=[]
    
    for i in range(3000):
        
        #print '************************\n'
        sim.doStep()
        #for agent in sim.agentList:
        #for agent in sim.agentList:
            
            #print 'agent position: ',agent.getAgentPosition()
        #print 'position: ',sim.agentList[0].getAgentPosition()
        #print 'position: ',sim.agentList[1].getAgentPosition()
        agentA.append(sim.agentList[0].getAgentPosition())
        agentB.append(sim.agentList[1].getAgentPosition())
        agentC.append(sim.agentList[2].getAgentPosition())
        agentD.append(sim.agentList[3].getAgentPosition())
        xA=[]
        yA=[]
        
        for coordinate in agentA:
            xA.append(coordinate[0])
            yA.append(coordinate[1])
            
        xB=[]
        yB=[]
        for coordinate in agentB:
            xB.append(coordinate[0])
            yB.append(coordinate[1])
            
        xC=[]
        yC=[]
        
        for coordinate in agentC:
            xC.append(coordinate[0])
            yC.append(coordinate[1])
            
        xD=[]
        yD=[]
        for coordinate in agentD:
            xD.append(coordinate[0])
            yD.append(coordinate[1])
            
        
        
    plt.plot(xA,yA,'ro')
    plt.plot(xB,yB,'bo')
    plt.plot(xC,yC,'go')
    plt.plot(xD,yD,'o')
            
    plt.axis([-2, 8, -2, 8])
    plt.show()
    '''
        
  