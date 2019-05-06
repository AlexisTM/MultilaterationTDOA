#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from scipy.optimize import minimize
from .geometry import Point, Circle
from .methods import LSEMethod
from time import time


class Anchor(object):
    def __init__(self, ID, position, measure = None):
        self.position = position
        self.ID = str(ID)
        self.last_seen = time()
        self.measure = measure

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        self._position = Point(position)

    def add_measure(self, data):
        self.measure = data
        self.last_seen = time()

    def valid(self, now = None, timeout = 0.5):
        if now is None:
            now = time()
        if self.measure is None:
            print("No measure yet... " + str(self))
            return False
        if now - self.last_seen > timeout:
            print("Last seen is too great! " + str(self.last_seen))
            return False
        return True

    def get(self):
        return Circle(self.position, self.measure)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "".join(['Anchor:', self.ID, '@', self.position.__str__()])


class Engine(object):
    def __init__(self, goal=[None, None, None], timeout=0.1):
        self.anchors = {}
        self.method = LSEMethod(goal)
        self.timeout = timeout

    def add_anchor(self, ID, position):
        """Add a certain ID"""
        if ID in self.anchors:
            self.anchors[ID].position = position
        else:
            self.anchors[ID] = Anchor(ID, position)

    def add_measure_id(self, ID, measure):
        """Add a measurement for a certain anchor ID"""
        if ID in self.anchors:
            self.anchors[ID].add_measure(measure)
        else:
            print("anchor " + str(ID) + " does not exist yet")

    def add_measure(self, position, measure, ID=None):
        """Distance measurement from an anchor position"""
        if ID is None:
            ID = str(position)

        if ID not in self.anchors:
            self.anchors[ID] = Anchor(ID, position, measure)
        else:
            self.anchors[ID] = measure

    def solve(self):
        cA = []
        for ID, anchor in self.anchors.items():
            if anchor.valid(self.timeout):
                cA.append(anchor.get())
        return self.method.solve(cA)
