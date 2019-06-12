#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
from multilateration_tdoa import DualTDoAEngine, Receiver, TDoAMeasurement, Anchor, Point
from math import sqrt, cos, sin
import numpy as np

NOISE = 0.25
MEAN_NOISE = 0.0
print("Configuration:")
print("-> Gaussian noise std:  ", NOISE)
print("-> Gaussian noise mean: ", MEAN_NOISE)
print("-> TDOA = |PB| - |PA| + noise()")

RECEIVERS_POSITION = {
    'frontleft': [ 0.35,  0.35, 0.5],
    'backright': [-0.35, -0.35, 0.5]
}

def noise():
    return np.random.normal(MEAN_NOISE, NOISE)

def tdoa(A,B,P):
    return P.dist(B)-P.dist(A) + noise()

def fake_TDOA(A,B,P):
    return TDoAMeasurement(A, B, tdoa(A,B,P))

def rotate(Pb, theta):
    return Point([
        Pb[0]*cos(theta) - Pb[1]*sin(theta),
        Pb[0]*sin(theta) + Pb[1]*cos(theta),
        Pb[2]
    ])

def add_fake_multi_receiver(engine, A, B, P, receivers_positions, theta):
    for recv_name in receivers_positions.keys():
        recv = rotate(receivers_positions[recv_name], theta) + P
        measurement = fake_TDOA(A,B,recv)
        engine.add_measure(measurement, receiver_id=recv_name)

A = Anchor((3,3,0))
B = Anchor((-2,2,0))
C = Anchor((2,-4,0))
D = Anchor((-3,-2,0))
P = Point(5,3,0.5) # Baselink position
theta = 1.0 # Orientation of the baselink


DTE = DualTDoAEngine(n_measurements=8, n_keep=0, max_dist_hess=100)

for recv_name in RECEIVERS_POSITION:
    DTE.add_receiver(Receiver(recv_name, RECEIVERS_POSITION[recv_name], n_measurements=6))

add_fake_multi_receiver(DTE, A, B, P, RECEIVERS_POSITION, theta)
add_fake_multi_receiver(DTE, A, C, P, RECEIVERS_POSITION, theta)
add_fake_multi_receiver(DTE, A, D, P, RECEIVERS_POSITION, theta)
add_fake_multi_receiver(DTE, B, C, P, RECEIVERS_POSITION, theta)
add_fake_multi_receiver(DTE, B, D, P, RECEIVERS_POSITION, theta)
add_fake_multi_receiver(DTE, C, D, P, RECEIVERS_POSITION, theta)

result = DTE.solve(height=P.z)

print(result.position)
print(result.theta)

# def show_result(result, hess_inv, expected):
#     print("result       x: %.2f    y: %.2f    z: %.2f" % (result.x, result.y, result.z))
#     print("expected     x: %.2f    y: %.2f    z: %.2f" % (expected.x, expected.y, expected.z))
#     if len(hess_inv) == 3:
#         print("hessian_inv: x: %.2f    y: %.2f    z: %.2f: (this is estimated from data)" % (hess_inv[0][0], hess_inv[1][1], hess_inv[2][2]))
#     else:
#         print("hessian_inv: x: %.2f    y: %.2f             (this is estimated from data)" % (hess_inv[0][0], hess_inv[1][1]))
#     error = result.dist(expected)
#     print("real error:     %.4f      (this is computed from the simulation ground truth)" % error)

# engine = TDoAEngine(n_measurements=6, max_dist_hess=100) # Avoid value rejection.

# A = Anchor((3,3,0))
# B = Anchor((-2,2,0))
# C = Anchor((2,-4,0))
# D = Anchor((-3,-2,0))
# P = Point(0,0,0)

# engine.add(fakeTDOA(A, C, P))
# engine.add(fakeTDOA(A, B, P))
# engine.add(fakeTDOA(A, D, P))
# engine.add(fakeTDOA(B, C, P))
# engine.add(fakeTDOA(B, D, P))
# engine.add(fakeTDOA(C, D, P))

# print("\n\nSolve in 3D from anchors on the ground and tag on the ground")
# result, hess_inv = engine.solve()
# show_result(result, hess_inv, P)

# A = Anchor((3,3,1))
# B = Anchor((-2,2,2))
# C = Anchor((2,-4,3))
# D = Anchor((-3,-2,2))
# P = Point((1.58,-1.51,0.31))

# engine.add(fakeTDOA(A, C, P))
# engine.add(fakeTDOA(A, B, P))
# engine.add(fakeTDOA(A, D, P))
# engine.add(fakeTDOA(B, C, P))
# engine.add(fakeTDOA(B, D, P))
# engine.add(fakeTDOA(C, D, P))

# print("\n\nSolve in 3D from anchors between 1 and 3 meters, tag at 0.31cm. The first Z approximation is exact.")
# m = engine.get()
# engine.measurements = m
# engine.last_result = Point(0,0,0.31)
# result, hess_inv = engine.solve()
# show_result(result, hess_inv, P)

# print("\n\nSolve in 2D with fixed height at 0.31cm with anchors between 1 and 3 meters and tag at 0.31cm.")
# # Use the same measurements to have correlated results.
# engine.measurements = m
# engine.last_result = Point(0,0,0) # Reset it to avoid to be pre converged from last result.
# result, hess_inv = engine.solve_2D(0.31)
# show_result(result, hess_inv, P)
