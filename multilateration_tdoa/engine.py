#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import numpy as np
from scipy.optimize import minimize, LbfgsInvHessProduct
from .geometry import Point, Circle
from time import time
from collections import defaultdict
import rospy
from math import sin, cos, pi


class TDoAResult(object):
    def __init__(self, position, theta = 0, raw_result = None):
        self.position = position if type(position) == Point else  Point(position)
        self.theta = (theta + pi) % (2*pi)-pi

        if raw_result is not None:
            self.raw_result = raw_result

            if(type(self.raw_result.hess_inv) == LbfgsInvHessProduct):
                self.hess_inv = self.raw_result.hess_inv.todense()
            else:
                self.hess_inv = self.raw_result.hess_inv

            self.time = time()
        else:
            # This is a placeholder, should be considered as a not valid result.
            self.time = 0
            self.hess_inv = np.ones([3,3])

        self.hessian_dist = sum(sum(self.hess_inv))

    def np_2D(self):
        return np.array([self.position.x, self.position.y])

    def np_3D(self):
        return np.array([self.position.x, self.position.y, self.position.z])

    def compute_tdoa(self, A, B):
        return self.position.dist(B) - self.position.dist(A)

    def error_to_measure(self, tdoa_measurement):
        tdoa = self.compute_tdoa(tdoa_measurement.anchorA, tdoa_measurement.anchorB)
        return tdoa - tdoa_measurement.tdoa

    def measurement_validity(self, tdoa_measurement, distance_threshold = 1.0, time_threshold = 0.3):
        time_diff = abs(tdoa_measurement.time - self.time)
        if(time_diff > time_threshold): # If the result is old, we need to converge back
            # print("time threshold")
            return True
        error = self.error_to_measure(tdoa_measurement)
        if error < distance_threshold:
            return True
        else:
            return False


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
    def __init__(self, n_measurements=8, n_keep=0, max_dist_hess=0.5, time_threshold=0.3):
        self.measurements = []
        self.n_measurements = n_measurements
        self.n_keep = n_keep
        self.last_result = TDoAResult(Point(0,0,0))
        self.method = 'L-BFGS-P'
        self.max_dist_hess = max_dist_hess
        self.time_threshold = time_threshold

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
        """
        e = 0
        approx = Point(approx)
        approx.z = height
        for mea in measurements:
            error = mea.tdoa - (mea.anchorB.position.dist(approx) - mea.anchorA.position.dist(approx))
            e += error**2 # Squared error problem
        return e

    def solve_2D(self, height = 0.0):
        """Optimize the position for LSE using a fixed height."""
        measurements = self.get()
        approx = self.last_result.np_2D()
        result = minimize(self.cost_function_2D, approx, args=(measurements, height), method=self.method, jac=self.jacobian_2D)

        tdoa_result = TDoAResult(Point(result.x[0], result.x[1], height), raw_result=result)

        if tdoa_result.hessian_dist < self.max_dist_hess:
            self.last_result = tdoa_result

        return tdoa_result

    def add_solve_2D(self, measurement, height = 0.0, data_review=None):
        """
        Add a measurement and optimize the position for LSE using a fixed height.
        This gives the proper flow for using the library.
        """
        if not self.last_result.measurement_validity(measurement):
            # rospy.logfatal(measurement)
            # rospy.logfatal(self.last_result.compute_tdoa(measurement.anchorA, measurement.anchorB))
            # rospy.logfatal(self.last_result.error_to_measure(measurement))
            # rospy.loginfo(self.last_result.position)
            if data_review is not None:
                data_review.reject(measurement)
            # print("rejected")
            return None
        if data_review is not None:
            data_review.accept(measurement)
        self.add(measurement)
        self.prune()
        # print("ready?")
        if self.ready():
            return self.solve_2D(height)
            # print("solving")
        return None

    def ready(self):
        """Returns true if we have the expected number of measurements in our buffer."""
        # print(self.measurements)
        return len(self.measurements) >= self.n_measurements

    def prune(self):
        """
        Removing duplicate measurements
        Removing old measurements
        """
        now = time()
        filtered = []
        # We go reverse through the list to keep only the newest innovations
        for mea in reversed(self.measurements):
            if mea not in filtered:
                filtered.append(mea)
            if now - mea.time > self.time_threshold:
                # We expect the measurements to be ordered by time.
                # Thus, ALL other measurements are late because we got reversed in the list.
                break
        self.measurements = filtered

class Receiver(object):
    def __init__(self, ID, position_to_baselink=(0,0,0), n_measurements=4, n_keep=0, time_threshold=0.3):
        self.ID = str(ID)
        self.position = Point(position_to_baselink)
        self.measurements = []
        self.n_measurements = n_measurements
        self.n_keep = n_keep
        self.time_threshold = time_threshold

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
    
    def clear(self):
        if self.n_keep == 0:
            self.measurements = []
        else:
            self.measurements = self.measurements[-self.n_keep:]

    def prune(self):
        """
        Removing duplicate measurements
        Removing old measurements
        """
        now = time()
        filtered = []
        # We go reverse through the list to keep only the newest innovations
        for mea in reversed(self.measurements):
            if mea not in filtered:
                filtered.append(mea)
            if now - mea.time > self.time_threshold:
                # We expect the measurements to be ordered by time.
                # Thus, ALL other measurements are late because we got reversed in the list.
                break
        self.measurements = filtered

    def approx_at_anchor(self, estimate, height):
        ct = cos(estimate[2])
        st = sin(estimate[2])

        # Center of the AGV estimated
        P = Point(estimate[0], estimate[1], height)

        # Step in between because we will need this data later
        position_rotated = Point((self.position.x*ct - self.position.y*st,
                            self.position.x*st + self.position.y*ct,
                            self.position.z))

        measurement_origin = position_rotated+P
        print(measurement_origin)
        print(P)
        return position_rotated, measurement_origin

    def ready(self):
        """Returns true if we have the expected number of measurements in our buffer."""
        # print(self.measurements)
        return len(self.measurements) >= self.n_measurements

    def jacobian(self, estimate, height):
        # According to: https://github.com/AlexisTM/MultilaterationTDOA/blob/master/Algorithmics.md
        # print("self.measurements", self.measurements)
        position_rotated, measurement_origin = self.approx_at_anchor(estimate, height)

        # print("JS1 XXXXXXXXXXX position_rotated", position_rotated)
        jacobian = np.array([0.0, 0.0, 0.0])
        for mea in self.measurements:
            PA = measurement_origin.dist(mea.anchorA)
            PB = measurement_origin.dist(mea.anchorB)
            mult = 2*(PB-PA-mea.tdoa)
            jacobian[0] += ((measurement_origin.x-mea.anchorB.position.x)/PB - (measurement_origin.x-mea.anchorA.position.x)/PA)*mult
            jacobian[1] += ((measurement_origin.y-mea.anchorB.position.y)/PB - (measurement_origin.y-mea.anchorA.position.y)/PA)*mult

            # print("JI1 measurement_origin.x", measurement_origin.x)
            # print("JI2 mea.anchorB.position.x", mea.anchorB.position.x)
            # print("JI3 position_rotated.y", position_rotated.y)
            # print("JI4 position_rotated", position_rotated)
            theta_j  = (-position_rotated.y*(measurement_origin.x-mea.anchorB.position.x) + position_rotated.x*(measurement_origin.y-mea.anchorB.position.y))/PB
            print(theta_j)
            theta_j -= (-position_rotated.y*(measurement_origin.x-mea.anchorA.position.x) + position_rotated.x*(measurement_origin.y-mea.anchorA.position.y))/PA
            print(theta_j)
            theta_j *= mult
            print(theta_j)
            jacobian[2] += theta_j
        # print("JE1 BBBBBBBBBBB position_rotatedx", position_rotated.x)
        # print("JE2 BBBBBBBBBBB position_rotatedy", position_rotated.y)
        # print("JE3 BBBBBBBBBBB position_rotatedz", position_rotated.z)
        print("jacobian", jacobian)
        return jacobian

    def cost_function(self, estimate, height):
        """
        Cost function for the 3D (x, y, theta) problem.
        It returns the Sum(error^2) between the approximation and the measurements.
        """
        print("estimate", estimate)
        e = 0
        _, measurement_origin = self.approx_at_anchor(estimate, height)
        for mea in self.measurements:
            error = mea.tdoa - (mea.anchorB.position.dist(measurement_origin) - mea.anchorA.position.dist(measurement_origin))
            e += error**2 # Squared error problem
        return e

class DualTDoAEngine(object):
    def __init__(self, n_measurements=8, n_keep=0, max_dist_hess=0.5, time_threshold=0.3):
        self.last_result = TDoAResult([0,0,0])
        self.method = 'BFGS'
        self.max_dist_hess = max_dist_hess
        self.receivers = defaultdict(lambda: Receiver('default', (0,0,0), n_keep=n_keep, n_measurements=n_measurements, time_threshold=time_threshold))

    def add_receiver(self, receiver):
        self.receivers[receiver.ID] = receiver

    def add_measure(self, measurement, receiver_id = None):
        self.receivers[receiver_id].add(measurement)

    def add_solve(self, measurement, height, receiver_id = None):
        self.receivers[receiver_id].add(measurement)

    def cost_function(self, estimate, height):
        e = 0
        for receiver in self.receivers.values():
            print("ID:", receiver.ID)
            e += receiver.cost_function(estimate, height)
        return e

    def jacobian(self, estimate, height):
        jacobian = np.array([0.0, 0.0, 0.0])
        for receiver in self.receivers.values():
            print("ID:", receiver.ID)
            jacobian += receiver.jacobian(estimate, height)
        return jacobian

    def prune_ready(self):
        readyness = {}
        for receiver in self.receivers.values():
            print("ID:", receiver.ID)
            receiver.prune()
            readyness[receiver.ID] = receiver.ready()
        return readyness

    def solve(self, height):
        """Optimize the position for LSE using a fixed height."""
        estimate = self.last_result.np_3D()

        print("SO SOLVE")
        result = minimize(self.cost_function, estimate, args=(height), method=self.method, jac=self.jacobian)

        print("SOLVED")
        for receiver in self.receivers.values():
            receiver.clear() # Remove old data and keep n_keep
        print("CLEARED")

        tdoa_result = TDoAResult(Point(result.x[0], result.x[1], height), theta=result.x[2] , raw_result=result)

        if tdoa_result.hessian_dist < self.max_dist_hess:
            self.last_result = tdoa_result

        return tdoa_result

class Anchor(object):
    anchors = {}

    def __init__(self, position, ID=None):
        if ID is None:
            ID=self.strID(position)
        ID=str(ID)
        self.position = position
        self.ID = ID
        self.last_seen = time()

    def __new__(class_, position, ID=None):
        if ID is None:
            ID=class_.strID(position)
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

    @staticmethod
    def strID(position):
        return "(%.2f,%.2f,%.2f)" % (position) #.x, position.y, position.z)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "".join(['Anchor:', self.ID, '@', self.position.__str__()])
