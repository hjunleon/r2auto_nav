#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 12:55:24 2021

@author: facestomperx
"""

from geometry_msgs.msg import Twist
import numpy as np


class TfRobotMovement():
    def __init__(self, *args, **kwargs):
        self.typeOfTracking = kwargs.get("typeOfTracking", "tf")
        if (self.typeOfTracking == "tf"):
            self.currentFrame = kwargs.get("tf_frame", None)
        self.isFirstFrame = True
        self.curVelocity = self.setZeroTwist()
    def canDetectSpeed(self):
        return self.isFirstFrame
    
    def setDetectSpeed(self):
        self.isFirstFrame = False
    
    """Checks if given frame is a time in the future"""
    def isFutureFrame(self, frame):
        #print("Checking if future frame")
        thisTime = self.getTimeFromFrame(frame)
        latestTime = self.getTimeFromFrame(self.currentFrame)
        #print("Reftime: ", thisTime)
        #print("curTime: ", latestTime)
        #print(id(frame))
        #print(id(self.currentFrame))
        if (id(frame) == id(self.currentFrame)):
            print("SAME FREAKING FRAME")
            return False
        if (thisTime > latestTime):
            return True
        return False
    
    def updateFrame(self, newFrame):
        self.currentFrame = newFrame
        
    def getFrame(self):
        return self.currentFrame
    
    def getTimeFromFrame(self, frame):
        return frame.header.stamp.sec + frame.header.stamp.nanosec * (10 ** -9)
    
    def setZeroTwist(self):
        retTwist = Twist()
        retTwist.linear.x = 0.0 
        retTwist.linear.y = 0.0 
        retTwist.linear.z = 0.0
        
        retTwist.angular.x = 0.0 
        retTwist.angular.y = 0.0 
        retTwist.angular.z = 0.0
        return retTwist
        
    def getTimeInterval(self, t1, t2):
        return abs(t2-t1)
    
    def frameTimeDiff(self, referenceFrame):
        refTime = self.getTimeFromFrame(referenceFrame)#in seconds, not miliseconds... may need the nano seconds
        curTime = self.getTimeFromFrame(self.currentFrame)
        
        #print("Reftime: ", refTime)
        #print("curTime: ", curTime)
        return self.getTimeInterval(refTime, curTime)
    #get velocity from referenceFRame, which of type TransformStamped
    #http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/TransformStamped.html
    def myLookupTwist(self, referenceFrame):
        #print("Getting robot twist")
        if(self.isFirstFrame == True):
            print("This is the first frame")
            self.isFirstFrame = False
            self.updateFrame(referenceFrame)
            return self.setZeroTwist()
        
        if (self.isFutureFrame(referenceFrame) == False):
            print("Somehow current frame not future frame")
            return self.getCurrentVelocity()
        
        
        print("Can get lookup twist")
        retTwist = Twist()    
        refTrans = referenceFrame.transform.translation
        curTrans = self.currentFrame.transform.translation
        
        print("referenceFrame")
        print(referenceFrame)
        print("self.currentFrame")
        print(self.currentFrame)
        
        refRot = referenceFrame.transform.rotation
        curRot = self.currentFrame.transform.rotation
        
        #print(referenceFrame.header.stamp)
        
        refTime = self.getTimeFromFrame(referenceFrame)#in seconds, not miliseconds... may need the nano seconds
        curTime = self.getTimeFromFrame(self.currentFrame)
        #epsilon = 10 ** -6
        timeDiff = refTime - curTime #+ epsilon
        print("Time diff: ", timeDiff)
        
        retTwist.linear.x = (refTrans.x - curTrans.x) / timeDiff
        retTwist.linear.y = (refTrans.y - curTrans.y) / timeDiff 
        retTwist.linear.z = (refTrans.z - curTrans.z) / timeDiff
        print("retTwist.linear.x: ", retTwist.linear.x)
        print("retTwist.linear.y: ", retTwist.linear.y)
        print("retTwist.linear.z: ", retTwist.linear.z)
        
        
        retTwist.angular.x = (refRot.x - curRot.x) / timeDiff
        retTwist.angular.y = (refRot.y - curRot.y) / timeDiff 
        retTwist.angular.z = (refRot.z - curRot.z) / timeDiff
        
        self.setNewVelocity(retTwist)
        return retTwist
    
    def getCurrentVelocity(self):
        return self.curVelocity
    
    def setNewVelocity(self, twist):
        self.curVelocity = Twist
        
        
        
        
        
        
        
class OdomRobotMovement():
    def __init__(self):
        self.hasReferenceMsg = False
        self.curVelocity = self.setZeroTwist()
        self.twistCovariance = np.zeros(36,dtype="float")
        self.curTime = 0
        
        
    def updateFromOdomMsg(self, msg):
        if (not self.hasReferenceMsg):
            self.hasReferenceMsg = True
        self.setVelocity(msg.twist.twist)
        self.setCurrentTime(msg.header.stamp)
        return None
        
    def toSeconds(self,t):
        return t.sec + t.nanosec * (10 ** -9)
    
    def hasReference(self):
        return self.hasReferenceMsg
    
    def setCurrentTime(self, t):
        self.curTime = self.toSeconds(t)
    
    def getTimeDiff(self, refMsg):
        refTime = self.toSeconds(refMsg.header.stamp)
        return refTime - self.curTime
    
    def setZeroTwist(self):
        retTwist = Twist()
        retTwist.linear.x = 0.0 
        retTwist.linear.y = 0.0 
        retTwist.linear.z = 0.0
        
        retTwist.angular.x = 0.0 
        retTwist.angular.y = 0.0 
        retTwist.angular.z = 0.0
        return retTwist    
    
    def getVelocity(self):
        return self.curVelocity
    
    def setVelocity(self, twist):
        self.curVelocity = twist
        
    def getTwistCovariance(self):
        return self.twistCovariance
    def setTwistCovariance(self,covariance):
        self.twistCovariance = covariance