#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function
from multilateration_tdoa import TDoAEngine, TDoAMeasurement, Anchor, Point
from math import sqrt
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import random
random.seed()

NOISE = 0.25
MEAN_NOISE = 0.0
METHOD = 'BFGS'
N_MEASUREMENTS_GENERATED = 15
N_MEASUREMENTS_USED = 4
SHAPE_MIN = -5.0
SHAPE_MAX = 20.0
SHAPE_DELTA = 1.0
TIMES = 5
HESS_REJECTION = 10.0

A = Anchor((0,0,0.31))
B = Anchor((9.83,0.0,0.31))
C = Anchor((15.0,4.92,0.31))
D = Anchor((15.0,9.78,0.31))
E = Anchor((-5.0,9.57, 0.31))
F = Anchor((-5.0,4.80, 0.31))
G = Anchor((9.69,14.68, 0.31))
H = Anchor((0.30,14.68, 0.31))

anchors = [A,B,C,D,E,F,G,H]

print("Configuration:")
print("-> Gaussian noise std:          ", NOISE)
print("-> Gaussian noise mean:         ", MEAN_NOISE)
print("-> min number of measures:      ", N_MEASUREMENTS_USED)
print("-> number of measures generated:", N_MEASUREMENTS_GENERATED)
print("-> TDOA = |PB| - |PA| + noise()")
print("-> Optimization method:", METHOD)
print("-> Number of computations:", TIMES)
print("-> Space shape:", "[",SHAPE_MIN, ",", SHAPE_MAX, "] with steps of ", SHAPE_DELTA)
print("-> Hessian sum rejection threshold (sum(sum(hess_inv)):", HESS_REJECTION)


def noise():
    return np.random.normal(MEAN_NOISE, NOISE)

def tdoa(A,B,P):
    return P.dist(B)-P.dist(A) + noise()

def fakeTDOA(A,B,P):
    return TDoAMeasurement(A, B, tdoa(A,B,P))

def generate_data(position, generated_engine, n=6):
    """Generates random measurements between random anchors"""
    P = Point(position)
    while n > 0:
        generated_engine.add(fakeTDOA(
            random.choice(anchors),
            random.choice(anchors),P))
        n -= 1
    return generated_engine

def compute_error(position, times=1, height=0, method='BFGS'):
    """Compute the error N times for the 2D optimization using the given method and height"""
    error = 0
    n = times
    point = Point(position)
    n_bad_data = 0
    generated_engine = TDoAEngine(n_measurements=N_MEASUREMENTS_USED)
    while n > 0:
        generate_data(position, generated_engine, N_MEASUREMENTS_GENERATED)
        generated_engine.prune()
        result, hess_inv = generated_engine.solve_2D(height=height)
        result.z = height
        hess_size = sum(sum(hess_inv))
        if hess_size > HESS_REJECTION:
            n_bad_data += 1
        else:
            error += point.dist(result)**2
            n = n-1
        if n_bad_data > 3:
            return None

    error /= (times - n_bad_data)
    return min(error, 2.0)


def plot_anchors(plt):
    x = []
    y = []
    for anchor in anchors:
        x.append(anchor.position.x)
        y.append(anchor.position.y)
    plt.scatter(x, y, s=100, marker='X', c='g')

def plot_all():
    x = y = np.arange(SHAPE_MIN, SHAPE_MAX, SHAPE_DELTA)

    Z = np.zeros((len(x), len(y)))
    X = np.zeros((len(x), len(y)))
    Y = np.zeros((len(x), len(y)))

    for i, xi in enumerate(x):
        for j, yj in enumerate(y):  
            Z[i, j] = compute_error((xi, yj, 0.31), height=0.31, times=5, method=METHOD)
            X[i, j] = xi
            Y[i, j] = yj
        print(str(int(i/len(x)*100))+"%")

    fig, ax = plt.subplots()
    im = ax.imshow(Z, cmap=cm.jet,
                origin='lower', extent=[SHAPE_MIN, SHAPE_MAX, SHAPE_MIN, SHAPE_MAX],
                vmax=1.0, vmin=0.0)

    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("Hessian intensity", rotation=-90, va="bottom")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    plot_anchors(plt)

    # Bilinear interpolation
    # fig, ax = plt.subplots()
    # im = ax.imshow(Z, cmap=cm.jet, interpolation='bilinear',
    #             origin='lower', extent=[SHAPE_MIN, SHAPE_MAX, SHAPE_MIN, SHAPE_MAX],
    #             vmax=1.0, vmin=0.0)

    # cbar = ax.figure.colorbar(im, ax=ax)
    # cbar.ax.set_ylabel("Hessian intensity", rotation=-90, va="bottom")
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # plot_anchors(plt)

    # Bicubic interpolation
    # fig, ax = plt.subplots()
    # im = ax.imshow(Z, cmap=cm.jet, interpolation='bicubic',
    #             origin='lower', extent=[SHAPE_MIN, SHAPE_MAX, SHAPE_MIN, SHAPE_MAX],
    #             vmax=1.0, vmin=0.0)

    # cbar = ax.figure.colorbar(im, ax=ax)
    # cbar.ax.set_ylabel("Hessian intensity", rotation=-90, va="bottom")
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # plot_anchors(plt)

    plt.show()

plot_all()

