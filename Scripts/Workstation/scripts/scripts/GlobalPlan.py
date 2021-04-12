#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 11:52:03 2021

@author: facestomperx
"""
class GlobalPlan():
    def __init__(self, poses):
        self.poses = poses
        self.curIdx = 0
    def getCurrentIndex(self):
        return self.curIdx
    def getByIdx(self, idx):
        if (idx >= self.getPointsLength()):
            idx = self.getPointsLength() - 1
        return self.poses[idx]
    
    def setCurrentIndex(self, idx):
        self.curIdx = idx
    
    def getPointsLength(self):
        return len(self.poses)
    
    def getCurrentGoalPose(self):
        #print("Getting Current Goal Pose")
        #print(self.curIdx, self.getPointsLength())
        
        return self.getByIdx(self.curIdx).pose.position
    
    def getCurrentGoalPosestamp(self):
        return self.getByIdx(self.curIdx)
    
    def getFinalGoalPose(self):
        return self.getByIdx(self.getPointsLength() - 1).pose.position
    
    def getFinalGoalPosestamp(self):
        return self.getByIdx(self.getPointsLength() - 1)
    
    def getAllPosestamps(self):
        return self.poses
    
    def getAllPoses(self):
        #print([miao.pose.position for miao in self.poses])
        return [miao.pose.position for miao in self.poses]
    
"""
    Has the start and end index of the global plan which is important for planning
    the next window. Also better for updating the same plan
"""
class TransformedPlan(GlobalPlan):
    def __init__(self, poses, startIdx, endIdx):
        super().__init__(poses)
        self.startIdx = startIdx
        self.endIdx = endIdx
    def getGlobalStartIdx(self):
        return self.startIdx
    def getGlobalEndIdx(self):
        return self.endIdx
    
    def setGlobalStartIdx(self, idx):
        self.startIdx = idx
    
    def setGlobalEndIdx(self, idx):
        self.endIdx = idx
        
    