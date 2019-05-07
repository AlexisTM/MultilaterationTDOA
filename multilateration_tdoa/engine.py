#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from scipy.optimize import minimize
from .geometry import Point, Circle
from .methods import LSEMethod
from time import time
from collections import defaultdict

class TDoAMeasurement(object):
    def __init__(self, anchorA, anchorB, tdoa, _time=None):
        self.time = _time or time()
        self.tdoa = tdoa
        self.anchorA = anchorA
        self.anchorB = anchorB


class TDoAEngine(object):
    def __init__(self, goal=[None, None, None], n_measurements=8):
        self.measurements = []
        self.n_measurements = n_measurements
        self.goal = goal
        self.last_result = Point(0,0,0)

    def add(self, measurement):
        self.measurements.append(measurement)

    def add_get(self, measurement, n_measurements = 8):
        self.add(measurement)
        if len(self.measurements) >= self.n_measurements:
            return self.get()

    def get(self):
        measures = self.measurements
        self.measurements = []
        return measures

    def cost_function(self, last_result, measurements):
        e = 0
        for mea in measurements:
            # print(mea.anchorA.position, last_result)
            error = mea.tdoa - (mea.anchorA.position.dist(last_result) - mea.anchorB.position.dist(last_result))
            # print(error, mea)
            e += error**2
        return e

    def solve(self):
        measurements = self.get()
        approx = np.array([self.last_result.x, self.last_result.y, self.last_result.z])
        result = minimize(self.cost_function, approx, args=(measurements), method='BFGS')
        ans = list(result.x)
        self.last_result = Point(ans)
        return Point(ans)

    def cost_function_2D(self, last_result, measurements, height):
        e = 0
        # other = Point(last_result.append(height))
        print(last_result)
        other = Point(last_result)
        other.z = height
        for mea in measurements:
            # print(mea.anchorA.position, last_result)
            error = mea.tdoa - (mea.anchorA.position.dist(other) - mea.anchorB.position.dist(other))
            # print(error, mea)
            e += error**2
        return e

    def solve_2D(self, height):
        measurements = self.get()
        approx = np.array([self.last_result.x, self.last_result.y])
        result = minimize(self.cost_function_2D, approx, args=(measurements, height), method='BFGS')
        ans = list(result.x)
        self.last_result = Point(ans)
        return Point(ans)


class Anchor(object):
    anchors = {}

    def __init__(self, position, ID=None):
        if ID is None:
            ID=str(position)
        ID=str(ID)
        self.position = position
        self.ID = ID
        self.last_seen = time()

    def __new__(class_, position, ID=None):
        if ID is None:
            ID=str(position)
        ID=str(ID)

        if ID in class_.anchors:
            class_.anchors[ID].position = position
        else:
            class_.anchors[ID] =  object.__new__(class_, position, ID)
        return class_.anchors[ID]

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
