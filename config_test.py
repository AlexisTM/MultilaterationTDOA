#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
from multilateration_tdoa import TDoAEngine, TDoAMeasurement, Anchor, Point
from math import sqrt
import random
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from multiprocessing import Pool
import tqdm

random.seed()

NOISE = 0.25

def noise():
    return random.random()*NOISE

def tdoa(A,B,P):
    return P.dist(B)-P.dist(A) + noise()

def fakeTDOA(A,B,P):
    return TDoAMeasurement(A, B, tdoa(A,B,P))

# A = Anchor((0,0,0.52))
# B = Anchor((9.83,0.0,1.75))
# C = Anchor((9.69,4.92,1.03))
# D = Anchor((9.69,9.78,1.79))
# E = Anchor((0.23,9.57, 0.5))
# F = Anchor((0.14,4.80, 1.76))
# G = Anchor((9.69,14.68, 1.5))
# H = Anchor((0.30,14.68, 1.5))
A = Anchor((0,0,0.31))
B = Anchor((9.83,0.0,0.31))
C = Anchor((15.0,4.92,0.31))
D = Anchor((15.0,9.78,0.31))
E = Anchor((-5.0,9.57, 0.31))
F = Anchor((-5.0,4.80, 0.31))
G = Anchor((9.69,14.68, 0.31))
H = Anchor((0.30,14.68, 0.31))

anchors = [A,B,C,D,E,F,G,H]

METHODS = ['Nelder-Mead',
           'Powell',
           'CG',
           'BFGS',
           'Newton-CG',
           'L-BFGS-B',
           'TNC',
           'COBYLA',
           'SLSQP',
           'trust-constr',
           'dogleg',
           'trust-ncg',
           'trust-krylov',
           'trust-exact']

METHOD = 'BFGS'
N_MEASUREMENTS = 15

def test(engine, measurements, method='BFGS'):
    engine.measurements = measurements
    result, hess_inv = engine.solve_2D(height = 0, method = method)
    print(result)
    print(method, str(hess_inv))
    errors[method] += P.dist(result)**2


def test_all(engine, measurements, times):
    n = times
    while n > 0:
        for method in METHODS:
            try:
                test(engine, measurements, method)
            except Exception as e:
                # print e
                pass
        n -= 1
    for method in errors.keys():
        print times
        errors[method] /= float(times)

def generate_data(position, generated_engine, n=6):
    P = Point(position)
    while n > 0:
        generated_engine.add(fakeTDOA(
            random.choice(anchors),
            random.choice(anchors),P))
        n -= 1
    return generated_engine


def compute_error(position, times=1, height=0, method='BFGS'):
    error = 0
    n = times
    point = Point(position)
    # print point, position
    n_bad_data = 0
    generated_engine = TDoAEngine(goal=[None, None, None], n_measurements=N_MEASUREMENTS)
    while n > 0:
        generate_data(position, generated_engine, N_MEASUREMENTS)
        generated_engine.prune()
        result, hess_inv = generated_engine.solve_2D(height=height)
        result.z = height
        det = np.linalg.det(hess_inv)
        if det > 10.0:
            n_bad_data += 1
        else:
            error += point.dist(result)**2
            # print "point:", point, " result:", result, " error:", error
            n = n-1
        if n_bad_data > 3:
            return None

    error /= (times - n_bad_data)
    # print error
    return min(error, 2.0)


def pool_compute(i):
    global x, y, Z, X, Y
    for j, yj in enumerate(y):
        Z[i, j] = compute_error((x[i], yj, 0), times=2)


def plot_anchors(plt):
    x = []
    y = []
    for anchor in anchors:
        x.append(anchor.position.x)
        y.append(anchor.position.y)
    plt.scatter(x, y, s=100, marker='X', c='g')

def plot_all():
    global x, y, Z, X, Y
    delta = 1.0
    MIN = -10.0
    MAX = 25.0
    x = y = np.arange(MIN, MAX, delta)
    # print x.shape
    # print y.shape

    Z = np.zeros((len(x), len(y)))
    X = np.zeros((len(x), len(y)))
    Y = np.zeros((len(x), len(y)))
    # pool = Pool(processes=8)
    # # pool.imap_unordered(pool_compute, x)
    # print range(len(x))
    # for _ in tqdm.tqdm(pool.imap_unordered(pool_compute, range(len(x))), total=len(x)):
    #     pass

    for i, xi in enumerate(x):
        for j, yj in enumerate(y):  
            Z[i, j] = compute_error((xi, yj, 0.31), height=0.31, times=5, method=METHOD)
            X[i, j] = xi
            Y[i, j] = yj
            # print Z[i,j]
        print str(int(i/len(x)*100))+"%"

    fig, ax = plt.subplots()
    im = ax.imshow(Z, cmap=cm.jet,
                origin='lower', extent=[MIN, MAX, MIN, MAX],
                vmax=1.0, vmin=0.0)

    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("Covariance (m*m)", rotation=-90, va="bottom")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    plot_anchors(plt)

    fig, ax = plt.subplots()
    im = ax.imshow(Z, cmap=cm.jet, interpolation='bilinear',
                origin='lower', extent=[MIN, MAX, MIN, MAX],
                vmax=1.0, vmin=0.0)

    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("Covariance (m*m)", rotation=-90, va="bottom")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    plot_anchors(plt)

    fig, ax = plt.subplots()
    im = ax.imshow(Z, cmap=cm.jet, interpolation='bicubic',
                origin='lower', extent=[MIN, MAX, MIN, MAX],
                vmax=1.0, vmin=0.0)

    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("Covariance (m*m)", rotation=-90, va="bottom")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    plot_anchors(plt)

    plt.show()

plot_all()

# test_all(engine, measurements, 1)
# print errors

# import IPython
# IPython.embed()
