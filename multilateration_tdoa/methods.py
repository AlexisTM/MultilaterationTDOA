#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from scipy.optimize import minimize
from .geometry import Point

from sys import version_info
if (version_info > (3, 0)):
    xrange = range


class Method(object):
    def __init__(self, goal):
        self.goal = goal
        self.last_result = Point(0,0,0)

    def update(self, cA):
        raise NotImplementedError


class LSEMethod(Method):
    def __init__(self, goal):
        Method.__init__(self, goal)

    def cost_function(self, x, c, r):
        e = 0
        # Other ways to converge to a certain axis value
        # Force an axis
        # print x
        # # This force an axis value
        # for i, value in enumerate(goal):
        #     if value is not None:
        #         x[i] = value

        # current = [0,0,0]
        # for i, value in enumerate(goal):
        #     if value is not None:
        #         current[i] = value
        #     else:
        #         current[i] = x[i]

        for i in xrange(len(c)):
            e += (c[i].dist(x)- r[i]) ** 2
        return e

    def solve(self, cA):
        # cA is a cicle array [Circle(), Circle()] representing measurements
        # l = number of circles
        l = len(cA)
        # r = radiuses of the circles (distance measured)
        r = [w.r for w in cA]
        # c = Point(), center of the circles (anchor position)
        c = [w.c for w in cA]
        # S = the sum of all radiuses
        S = sum(r)
        # W = Normalized 1/distances [(Sum - distance) / (Nmeasures-1)*Sum] 
        W = [(S - w) / ((l - 1) * S) for w in r]
        p0 = self.last_result  # Initialized Point
        for i in range(l):
            # p0 += Normalized distance * centers
            p0 = p0 + W[i] * c[i]
            x0 = np.array([p0.x, p0.y, p0.z])
        # self.cost_function = the function to be minimized
        # x0 = the initial estimation that gets iterated
        # Extra arguments: 
        #   c = Point(), anchor positions
        #   r = all measurements
        for i, value in enumerate(self.goal):
            if value is not None:
                x0[i] = value

        result = minimize(self.cost_function, x0, args=(c, r), method='BFGS')
        ans = list(result.x)
        return Point(ans)
