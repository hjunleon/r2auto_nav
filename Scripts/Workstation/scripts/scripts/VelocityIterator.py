#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 23:09:27 2021

@author: facestomperx
"""
class VelocityIterator():
    def __init__(self, minVel, maxVel, num_samples):
        #print("Init velocity iterator")
        self._samples = []
        #print("minVel vs maxVel:",minVel, maxVel)
        if (minVel == maxVel):
            self._samples.append(minVel)
        else:
            num_samples = max(2, num_samples)
            step_size = (maxVel-minVel) / float(max(1, (num_samples - 1)))
            #print("Step_Size: ", step_size)
            #print("Num of samples: ", num_samples)
            nextVel = minVel
            current = nextVel
            for j in range(num_samples):
                current = nextVel
                #print("Current Vel ", current)
                nextVel += step_size
                self._samples.append(current)
                if (current < 0 and nextVel > 0):
                    self._samples.append(0.0)
            self._samples.append(maxVel)
    def __iter__(self):
        self._current_index = 0
        return self
    def __next__(self):
        if self._current_index < len(self._samples):
            x = self._samples[self._current_index]
            self._current_index += 1
            return x
        else:
            raise StopIteration
            
    def resetIdx(self):
        self._current_index = 0