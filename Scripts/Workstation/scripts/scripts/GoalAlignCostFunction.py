#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 00:47:39 2021

@author: facestomperx
"""
import utils
#https://github.com/ros-planning/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_critics/src/goal_align.cpp
from TrajectoryCostFunction import TrajectoryCostFunction 
class PathDistCostFunction(TrajectoryCostFunction):
    def __init__(self, *args, **kwargs):
        super().__init__(kwargs.get("scale"), kwargs.get("costmap"))
        
    def updateGlobalPlan(self, global_plan):
        self.global_plan = global_plan
        
    def scoreTrajectory(self,traj):
        traj_goal = traj[len(traj) - 1]
        local_plan_goal = self.global_plan[len(self.global_plan) - 1]
        return self.getManhattanDistance([local_plan_goal],[traj_goal])