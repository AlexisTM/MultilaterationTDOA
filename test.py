#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
from multilateration_tdoa import TDoAEngine, TDoAMeasurement, Anchor, Point
from math import sqrt
import numpy as np

NOISE = 0.25
MEAN_NOISE = 0.0
print("Configuration:")
print("-> Gaussian noise std:  ", NOISE)
print("-> Gaussian noise mean: ", MEAN_NOISE)
print("-> TDOA = |PB| - |PA| + noise()")

def noise():
    return np.random.normal(MEAN_NOISE, NOISE)

def tdoa(A,B,P):
    return P.dist(B)-P.dist(A) + noise()

def fakeTDOA(A,B,P):
    return TDoAMeasurement(A, B, tdoa(A,B,P))

def show_result(result, hess_inv, expected):
    print("result       x: %.2f    y: %.2f    z: %.2f" % (result.x, result.y, result.z))
    print("expected     x: %.2f    y: %.2f    z: %.2f" % (expected.x, expected.y, expected.z))
    if len(hess_inv) == 3:
        print("hessian_inv: x: %.2f    y: %.2f    z: %.2f: (this is estimated from data)" % (hess_inv[0][0], hess_inv[1][1], hess_inv[2][2]))
    else:
        print("hessian_inv: x: %.2f    y: %.2f             (this is estimated from data)" % (hess_inv[0][0], hess_inv[1][1]))
    error = result.dist(expected)
    print("real error:     %.4f      (this is computed from the simulation ground truth)" % error)

engine = TDoAEngine(n_measurements=6, max_dist_hess=100) # Avoid value rejection.

A = Anchor((3,3,0))
B = Anchor((-2,2,0))
C = Anchor((2,-4,0))
D = Anchor((-3,-2,0))
P = Point(0,0,0)

engine.add(fakeTDOA(A, C, P))
engine.add(fakeTDOA(A, B, P))
engine.add(fakeTDOA(A, D, P))
engine.add(fakeTDOA(B, C, P))
engine.add(fakeTDOA(B, D, P))
engine.add(fakeTDOA(C, D, P))

print("\n\nSolve in 3D from anchors on the ground and tag on the ground")
result, hess_inv = engine.solve()
show_result(result, hess_inv, P)

A = Anchor((3,3,1))
B = Anchor((-2,2,2))
C = Anchor((2,-4,3))
D = Anchor((-3,-2,2))
P = Point((1.58,-1.51,0.31))

engine.add(fakeTDOA(A, C, P))
engine.add(fakeTDOA(A, B, P))
engine.add(fakeTDOA(A, D, P))
engine.add(fakeTDOA(B, C, P))
engine.add(fakeTDOA(B, D, P))
engine.add(fakeTDOA(C, D, P))

print("\n\nSolve in 3D from anchors between 1 and 3 meters, tag at 0.31cm. The first Z approximation is exact.")
m = engine.get()
engine.measurements = m
engine.last_result = Point(0,0,0.31)
result, hess_inv = engine.solve()
show_result(result, hess_inv, P)

print("\n\nSolve in 2D with fixed height at 0.31cm with anchors between 1 and 3 meters and tag at 0.31cm.")
# Use the same measurements to have correlated results.
engine.measurements = m
engine.last_result = Point(0,0,0) # Reset it to avoid to be pre converged from last result.
result, hess_inv = engine.solve_2D(0.31)
show_result(result, hess_inv, P)
