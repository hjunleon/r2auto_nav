#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  4 17:33:11 2021

@author: facestomperx
"""
import utils
from types import SimpleNamespace

#https://github.com/ros-planning/navigation/blob/8c4933517b56a9ac8f193068df4d9ff333f21614/base_local_planner/src/obstacle_cost_function.cpp
#https://github.com/ros-planning/navigation/blob/8c4933517b56a9ac8f193068df4d9ff333f21614/base_local_planner/include/base_local_planner/obstacle_cost_function.h

from TrajectoryCostFunction import TrajectoryCostFunction 
class ObstacleCostFunction(TrajectoryCostFunction):
    def __init__(self, *args, **kwargs):
        super().__init__(kwargs.get("scale"), kwargs.get("costmap"))
        self._max_trans_vel = kwargs.get("max_trans_vel",0.0)
        self._max_scaling_factor = kwargs.get("max_scaling_factor",1.0)
        self._scaling_speed = kwargs.get("scaling_speed",1.0)
        self._to_sum_scores = kwargs.get("sum_scores", True) #makes sense since I want the traj's point by point score to be summed
        
    
    def updateCostMap(self, costmap):
        self._costmap = costmap
        
        
    def scoreTrajectory(self,traj):
        #print("Scoring obstacle cost")
        cost = 0
        scale = self.getScalingFactor(traj,self._scaling_speed, self._max_trans_vel, self._max_scaling_factor)
        for i in range(traj.getPointsListLength()):
            point = SimpleNamespace()
            traj.getPoint(i,point)
            f_cost = self.footprintCost(point, scale)
            if (f_cost < 0):
                return f_cost #for my case i should just return infinity
            
        #print(cost)
        return cost
        
    def getScalingFactor(self, traj, scaling_speed, max_trans_vel, max_scaling_factor):
        v_mag = utils.hypotenuseLen(traj._xv, traj._yv)
        # if over certain speed threshold, scale robot footprint
        # to slow down or avoid walls?
        #print("GET SCALING FACTOR NANI")
        #print(v_mag)
        #print(scaling_speed)
        scale = 1.0
        if (v_mag > scaling_speed):
            ratio = (v_mag - scaling_speed) / (max_trans_vel - scaling_speed)
            scale = max_scaling_factor * ratio + 1.0
        return scale
    
    """
    Ok alright so, this is a pretty important discovery haha. Apparently this checks whether at this part of 
    the trajectory, what's the cost involved in terms of obstacles in the way and if there aren't any obstacles,
    what's the cost associated?
    
    Unlike navstack, i  do not want costmap , world_model and footprint_specification to be paramters. They should be 
    made only gotten by the function that needs it at the time, which is costmap
    """
    
    
    def footprintCost(self,pointObj, scale):
        #print("Getting footprint cost")
        #print(self._costmap)
        footprint_cost = self._costmap.footPrintCostRadius(pointObj)#0#self._costmap.footPrintCost(pointObj, None) #fix this later
        if (footprint_cost < 0):
            return -6
        if(not self._costmap.isPointValid(pointObj)):
            return -7.0
        #print(pointObj)
        cell = self._costmap.worldToMap(pointObj)
        cellX,cellY = cell[0],cell[1]
        #print("Cell coords: ")
        #print(cell)
        return max(max(0.0, footprint_cost), self._costmap.getCost(cellX,cellY)) 
        