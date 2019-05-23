#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from scipy.optimize import minimize, LbfgsInvHessProduct
from .geometry import Point, Circle
from time import time
from collections import defaultdict


class TDoAMeasurement(object):
    def __init__(self, anchorA, anchorB, tdoa, _time=None):
        self.time = _time or time()
        self.tdoa = tdoa
        self.anchorA = anchorA
        self.anchorB = anchorB

    def __str__(self):
        return "".join([
            '\nA: ', str(self.anchorA.position),
            '\nB: ', str(self.anchorB.position),
            '\ntdoa: ', str(self.tdoa)
        ])
    
    def __eq__(self, other):
        if self.anchorA.ID == other.anchorA.ID and self.anchorB.ID == other.anchorB.ID:
            return True
        if self.anchorA.ID == other.anchorB.ID and self.anchorB.ID == other.anchorA.ID:
            return True
        return False

    def __repr__(self):
        return self.__str__()


class TDoAEngine(object):
    def __init__(self, n_measurements=8, n_keep=0, max_dist_hess=0.5):
        self.measurements = []
        self.n_measurements = n_measurements
        self.n_keep = n_keep
        self.last_result = Point(0,0,0)
        self.method = 'BFGS'
        self.max_dist_hess_squared = max_dist_hess

    def add(self, measurement):
        """Add a measurement in the array."""
        self.measurements.append(measurement)

    def get(self):
        """Returns the measurements and keep the self.n_keep last measurements."""
        measures = self.measurements
        if self.n_keep == 0:
            self.measurements = []
        else:
            self.measurements = measures[-self.n_keep:]
        return measures

    def cost_function(self, approx, measurements):
        """
        Cost function for the 3D problem
        TODO: Use weighed least square cost function
        """
        e = 0
        for mea in measurements:
            error = mea.tdoa - (mea.anchorB.position.dist(approx) - mea.anchorA.position.dist(approx))
            e += error**2
        return e

    def solve(self):
        """Optimize the position for LSE using in a 3D problem."""
        measurements = self.get()
        approx = np.array([self.last_result.x, self.last_result.y, self.last_result.z])
        result = minimize(self.cost_function, approx, args=(measurements), method=self.method)
        position = Point(list(result.x))

        if(type(result.hess_inv) == LbfgsInvHessProduct):
            hess_inv = result.hess_inv.todense()
        else:
            hess_inv = result.hess_inv
        dist = self.scalar_hess_squared(hess_inv)
        if dist < self.max_dist_hess_squared:
            self.last_result = position

        self.last_result = position
        return position, hess_inv

    def jacobian_2D(self, estimate, measurements, height):
        # According to: https://github.com/AlexisTM/MultilaterationTDOA/blob/master/Algorithmics.md
        jacobian = np.array([0.0, 0.0])
        approx = Point(estimate)
        approx.z = height
        for mea in measurements:
            PA = approx.dist(mea.anchorA)
            PB = approx.dist(mea.anchorB)
            mult = PB-PA-mea.tdoa
            jacobian[0] += 2*((approx.x-mea.anchorB.position.x)/PB - (approx.x-mea.anchorA.position.x)/PA)*mult
            jacobian[1] += 2*((approx.y-mea.anchorB.position.y)/PB - (approx.y-mea.anchorA.position.y)/PA)*mult
            # Doing it for each member separetely is about ~35% faster
            # j = 2*((approx-mea.anchorB.position)/PB - (approx-mea.anchorA.position)/PA)*(PB-PA-mea.tdoa)
            # jacobian[0] += j.x
            # jacobian[1] += j.y
        return jacobian

    def cost_function_2D(self, approx, measurements, height):
        """
        Cost function for the 2D problem.
        It returns the Sum(error^2) between the approximation and the measurements.
        TODO: Use weighed least square cost function
        """
        e = 0
        approx = Point(approx)
        approx.z = height
        for mea in measurements:
            error = mea.tdoa - (mea.anchorB.position.dist(approx) - mea.anchorA.position.dist(approx))
            e += error**2
        return e

    def solve_2D(self, height = 0.0):
        """Optimize the position for LSE using a fixed height to reduce problem complexity."""
        measurements = self.get()
        approx = np.array([self.last_result.x, self.last_result.y])
        result = minimize(self.cost_function_2D, approx, args=(measurements, height), method=self.method, jac=self.jacobian_2D)
        position = Point(list(result.x) + [height])
        if(type(result.hess_inv) == LbfgsInvHessProduct):
            hess_inv = result.hess_inv.todense()
        else:
            hess_inv = result.hess_inv
        dist = self.scalar_hess_squared(hess_inv)
        if dist < self.max_dist_hess_squared:
            self.last_result = position
        return position, hess_inv

    def add_solve_2D(self, measurement, height = 0.0):
        """
        Add a measurement and optimize the position for LSE using a fixed height.
        This gives the proper flow for using the library.
        """
        self.add(measurement)
        self.prune()
        if self.ready():
            return self.solve_2D(height)
        return None, None
    
    def ready(self):
        """Returns true if we have the expected number of measurements in our buffer."""
        return len(self.measurements) >= self.n_measurements

    def prune(self):
        """
        Removing the wrong measurements: Too old, or from/to the same anchors.
        TODO: Remove old measurements (higher than measurement timeout)
        """
        filtered = []
        for mea in reversed(self.measurements):
            if mea not in filtered:
                filtered.append(mea)
        self.measurements = filtered

    @staticmethod
    def scalar_hess_squared(hess):
        return hess[0][0]**2 + hess[0][1]**2 + hess[1][0]**2 + hess[1][1]**2

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

    def __getitem__(self, id):
        return self.position[id]

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
