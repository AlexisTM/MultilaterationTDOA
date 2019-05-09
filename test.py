#!/usr/bin/env python
# -*- coding: utf-8 -*-

from multilateration_tdoa import TDoAEngine, TDoAMeasurement, Anchor, Point
from math import sqrt
import random
random.seed()

NOISE = 0.25

def noise():
    return random.random()*NOISE

engine = TDoAEngine(goal=[None, None, None], n_measurements=6) # To fix that the resulting height should be 2 meters
# P=multilateration.Project() # for 3D, simply do not set a goal


A = Anchor((3,3,0))
B = Anchor((-2,2,0))
C = Anchor((2,-4,0))
D = Anchor((-3,-2,0))




engine.add(TDoAMeasurement(A, C, -0.23 + noise()))
engine.add(TDoAMeasurement(A, B, 1.41 + noise()))
engine.add(TDoAMeasurement(A, D, 0.64 + noise()))
engine.add(TDoAMeasurement(B, C, -1.64 + noise()))
engine.add(TDoAMeasurement(B, D, -0.78 + noise()))
engine.add(TDoAMeasurement(C, D, 0.87 + noise()))

result, hess_inv = engine.solve()
print(result)
expected = Point(0,0,0)
print("Error = ", expected.dist(result))
# Expected result = (0,0,0)


engine.add(TDoAMeasurement(A, C, 2.2 + noise()))
engine.add(TDoAMeasurement(A, B, -0.29 + noise()))
engine.add(TDoAMeasurement(A, D, 0.12 + noise()))
engine.add(TDoAMeasurement(B, C, 2.49 + noise()))
engine.add(TDoAMeasurement(B, D, 0.41 + noise()))
engine.add(TDoAMeasurement(C, D, -2.08 + noise()))

result, hess_inv = engine.solve()
print(result)
expected = Point(1.58,-1.51,0)
print("Error = ", expected.dist(result))
# Expected result P= (1.58,-1.51)



A = Anchor((3,3,1))
B = Anchor((-2,2,2))
C = Anchor((2,-4,3))
D = Anchor((-3,-2,2))

engine.add(TDoAMeasurement(A, C, 1.09 + noise()))
engine.add(TDoAMeasurement(A, B, -0.51 + noise()))
engine.add(TDoAMeasurement(A, D, -0.13 + noise()))
engine.add(TDoAMeasurement(B, C, 1.6 + noise()))
engine.add(TDoAMeasurement(B, D, 0.38 + noise()))
engine.add(TDoAMeasurement(C, D, -1.22 + noise()))

m = engine.get()
engine.measurements = m
engine.last_result.z = 0.31
# P= (1.58,-1.51,0.31)
result, hess_inv = engine.solve()

print(result)
expected = Point(1.58,-1.51,0.31)
print("Error3D = ", expected.dist(result))


engine.measurements = m
result, hess_inv = engine.solve_2D(0.31)
result.z = 0.31
print(result)
print("Error2D = ", expected.dist(result))

print(hess_inv)