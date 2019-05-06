#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
from math import pi, cos, sin, atan, acos
import numpy as np

class Circle(object):
    def __init__(self, p, r):
        self.c = p
        self.r = float(r)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "".join(['Circle[' + self.c.__str__() + ':' + str(self.r) + ']'])

    def __eq__(self, other):
        return self.c == other.c and self.r == other.r

class Point(object):
    def __init__(self, x, y=0, z=0):
        if(type(x) in [list, tuple, np.ndarray]):
            self.x = x[0] if len(x) >= 1 else 0.0
            self.y = x[1] if len(x) >= 2 else 0.0
            self.z = x[2] if len(x) >= 3 else 0.0
        else:
            self.x = x
            self.y = y
            self.z = z

    def __str__(self):
        return "".join(['Point(' + str(self.x) + ',' + str(self.y) + ',' + str(self.z) + ')'])

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def __sub__(self, other):
        if isinstance(other, Point):
            tx = self.x - other.x
            ty = self.y - other.y
            tz = self.z - other.z
        else:
            tx = self.x - other.dx
            ty = self.y - other.dy
            tz = self.z - other.dz
        return Point(tx, ty, tz)

    def __add__(self, other):
        if isinstance(other, Point):
            tx = self.x + other.x
            ty = self.y + other.y
            tz = self.z + other.z
        else:
            tx = self.x + other.dx
            ty = self.y + other.dy
            tz = self.z + other.dz
        return Point(tx, ty, tz)

    def __mul__(self, other):
        return Point(other * self.x, other * self.y, other * self.z)

    def __rmul__(self, other):
        return Point(other * self.x, other * self.y, other * self.z)

    def __div__(self, other):
        return Point(self.x / other, self.y / other, self.z / other)

    def __neg__(self):
        x = -self.x
        y = -self.y
        z = -self.z
        return Point(x, y, z)

    def __getitem__(self, id):
        return [self.x, self.y, self.z][id]

    def area(self):
        return 0.0

    def dist(self, other):
        return ((self[0] - other[0]) ** 2 + (self[1] - other[1]) ** 2 + (self[2] - other[2]) ** 2) ** 0.5

    def std(self):
        return [self.x, self.y, self.z]

    def c2s(self):
        R = self.dist(Point(0, 0, 0))
        lg = atan(self.y / self.x)
        lat = acos(self.z / R)
        return (lg, lat, R)

    def transform(self, p, rot):
        px = cos(rot) * self.x + sin(rot) * self.y
        py = -sin(rot) * self.x + cos(rot) * self.y
        p_t = Point(px, py)
        return p_t - p

    def rot(self, a):
        px = cos(a) * self.x - sin(a) * self.y
        py = sin(a) * self.x + cos(a) * self.y
        return Point(px, py)
