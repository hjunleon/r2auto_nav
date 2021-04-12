#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 00:47:39 2021

@author: facestomperx
"""
import utils

from TrajectoryCostFunction import TrajectoryCostFunction 
class GoalDistCostFunction(TrajectoryCostFunction):
    def __init__(self, *args, **kwargs):
        super().__init__(kwargs.get("scale"), kwargs.get("costmap"))
        print("Init Goal Dist")
        self.global_plan = kwargs.get("global_plan",None)
        print(self.global_plan)
    def updateGlobalPlan(self, global_plan):
        self.global_plan = global_plan
        
    def scoreTrajectory(self,traj):
       # print("Scoring Goal Distance")
        traj_points = traj.getAllPoints()
        #print(len(traj_points))
        #print(traj.getPointsListLength())
        traj_goal = traj_points[traj.getPointsListLength() - 1]
        #print(self.global_plan.getAllPosestamps())
        
        local_plan_goal = self.global_plan.getFinalGoalPose()#self.global_plan.getCurrentGoalPose() #EDIT: because Im passing pruned plans, then just get final goal pose
        
        
        return self.getManhattanDistance([local_plan_goal],[traj_goal])