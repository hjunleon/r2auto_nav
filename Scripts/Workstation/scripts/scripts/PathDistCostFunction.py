#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  5 23:09:06 2021

@author: facestomperx
"""
import utils

from TrajectoryCostFunction import TrajectoryCostFunction 
class PathDistCostFunction(TrajectoryCostFunction):
    def __init__(self, *args, **kwargs):
        super().__init__(kwargs.get("scale"), kwargs.get("costmap"))
        self.global_plan = kwargs.get("global_plan",None)
    def updateGlobalPlan(self, global_plan):
        self.global_plan = global_plan
        
    def scoreTrajectory(self,traj):
        #print("Scoring Path Distance")
        #print(self.global_plan.getAllPoses())
        return self.getManhattanDistance(self.global_plan.getAllPoses() ,traj.getAllPoints())
    
        