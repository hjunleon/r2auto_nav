#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  4 17:24:12 2021

@author: facestomperx
"""

import utils

class TrajectoryCostFunction():
    def __init__(self,scale, costmap):
        self._scale = scale
        self._costmap = costmap
    def getScale(self):
        return self._scale
    def setScale(self,scale):
        self._scale = scale
        
    def getCostmap(self):
        return self._costmap
    def setCostmap(self,costmap):
        self._costmap = costmap
        
    def getManhattanDistance(self, transformed_plan, trajectory):
        #print("GEtting Manhattan Distance: ", len(transformed_plan), len(trajectory))
        max_index = min(len(transformed_plan), len(trajectory))
        
        total_dist = 0.0
        for idx in range(max_index):
            total_dist += utils.getManhattenDistance(transformed_plan[idx], trajectory[idx])
        
        #print("TOtal manhattan distance: ", total_dist)
        return total_dist
    
    def getEuclideanDistance(self, transformed_plan, trajectory):
        max_index = min(len(transformed_plan), len(trajectory))
        total_dist = 0.0
        for idx in range(max_index):
            total_dist += utils.getEuclideanDistance(transformed_plan[idx], trajectory[idx])
        return total_dist
    
    #https://stackoverflow.com/questions/4714136/how-to-implement-virtual-methods-in-python
    def scoreTrajectory(traj):
        raise NotImplementedError()