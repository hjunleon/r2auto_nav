#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  5 13:48:56 2021

@author: facestomperx
"""
class LineIterator():
    """
    FYI num is numerator, den is denumerator
    """
    def __init__(self,p1,p2):
        self._p1 = p1
        self._p2 = p2
        self._curX = p1.x
        self._curY = p1.y
        self._deltaX = abs(p2.x - p1.x)
        self._deltaY = abs(p2.y - p1.y)
        self._curCellIdx = 0
        if (p2.x >= p1.x):
            self._xinc1 = 1
            self._xinc2 = 1
        else:
            self._xinc1 = -1
            self._xinc2 = -1
            
        if (p2.y >= p1.y):
            self._yinc1 = 1
            self._yinc2 = 1
        else:
            self._yinc1 = -1
            self._yinc2 = -1
            
        if (self._deltaX >= self._deltaY):
            # this is when there's more x values than y values
            self._xinc1 = 0
            self._yinc2 = 0
            self._den = self._deltaX
            self._num = self._deltaX / 2
            self._numAdd = self._deltaY
            self._numCells = self._deltaX
        else:
            self._xinc2 = 0
            self._yinc1 = 0
            self._den = self._deltaY
            self._num = self._deltaY / 2
            self._numAdd = self._deltaX
            self._numCells = self._deltaY
            
    def isValid(self):
        return self._curCellIdx <= self._numCells
    
    def advance(self):
        self._num += self._numAdd
        if (self._num >= self._den):
            self._num -= self._den
            self._curX += self._xinc1
            self._curY += self._yinc1
        self._curX += self._xinc2
        self._curY += self._yinc2
        
        self._curCellIdx += 1
        
    def getX(self):
        return self._curX
    def getY(self):
        return self._curY
    