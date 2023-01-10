#!/usr/bin/env python

import random
import logging
import abc
import numpy as np
import math 

__MAX_ITER = 5000
__MAX_ONE_ITERATION_PER_X_POINTS = 50
__MIN_POINTS_TO_CALC_FACTOR = 1.5
__RANDOM_SAMPLES = 5

logging.basicConfig(level=logging.DEBUG)

class Model(abc.ABC):
    @abc.abstractmethod
    def calculate_model_params(points: list):
        pass

    @abc.abstractmethod
    def fit_points_to_model(points: list):
        pass

    @abc.abstractmethod
    def get_params():
        pass

class Line(Model):
    def __init__(self):
        self.max_dist = 1

    def calculate_model_params(self, points: list):
        x = [point[0] for point in points]
        y = [point[1] for point in points]
        ab_params = np.polyfit(x, y, 1)
        a = ab_params[0]
        b = -1
        c = ab_params[1]
        self.params = (a,b,c)
        logging.debug(f"params: {self.params}")

    def fit_points_to_model(self, points: list):
        fitting_points = []
        for p in points:
            if self.__dist(p, self.params) < self.max_dist:
                fitting_points.append(p)
        logging.debug(f"fitting_points before 2nd param calc: {fitting_points}")
        self.calculate_model_params(fitting_points)
        # recalculate points for better fit
        fitting_points = []
        for p in points:
            if self.__dist(p, self.params) < self.max_dist:
                fitting_points.append(p)
        logging.debug(f"fitting_points after 2nd param calc: {fitting_points}")
        # split line into multiple lines if break in between
        fitting_points = self.__split_if_break_in_line(fitting_points, 10)
        logging.debug(f"fitting_points after split: {fitting_points}")
        return fitting_points

    def get_params(self):
        return self.params

    @staticmethod
    def __dist(point, params):
        dist = abs((params[0] * point[0] + params[1] * point[1] + params[2])) / (math.sqrt(params[0] * params[0] + params[1] * params[1]))
        return dist
    @staticmethod
    def __split_if_break_in_line(points, max_dist):
        indices = [i + 1 for (x, y, i) in zip(points, points[1:], range(len(points))) if max_dist < math.dist(x, y)]
        result = [points[start:end] for start, end in zip([0] + indices, indices + [len(points)])]
        return result

def remove_line_from_list(points: list, line: list):
    # delete points from line from points list
    tmp = [item for sublist in tmp for item in sublist]
    new_points = []
    for p in points:
        # not in wont work because reference vs copy check
        if p not in tmp:
            new_points.append(p)
    return new_points

def extract_line(points: list, threshold: int, model: Model):
    lines = []
    iterations = min(__MAX_ITER, len(points)/__MAX_ONE_ITERATION_PER_X_POINTS)
    for i in range(iterations):
        if len(points) < threshold*__MIN_POINTS_TO_CALC_FACTOR:
            break
        central_point = random.randint(0, len(points))
        left_index = central_point - int(threshold/2)
        right_index = left_index + threshold
        if left_index < 0:
            left_index = 0
            right_index = threshold
        if right_index >= len(points):
            right_index = len(points)-1
            left_index = right_index - threshold

        if left_index < 0:
            logging.error("left index under 0")
            raise Exception("left_index under 0")
        
        num_samples = __RANDOM_SAMPLES
        if threshold < num_samples:
            num_samples = threshold/2
        # random_points = random.sample(points[left_index:right_index], num_samples)
        random_points_idx = random.sample(range(left_index, right_index), num_samples)
        logging.debug("random points range: {}-{}".format(left_index, right_index))
        logging.debug("random points indices: {}".format(random_points_idx))
        model.calculate_model_params([points[idx] for idx in random_points_idx])
        tmp = model.fit_points_to_model(points)
        lines.append(tmp)
        # delete points from line from points list
        tmp = [item for sublist in tmp for item in sublist]
        new_points = []
        for p in points:
            # not in wont work because reference vs copy check
            if p not in tmp:
                new_points.append(p)
        points = new_points

if __name__ == "__main__":
    model = Line()
    points = [(1.1,0.5), (0.9,1), (1,3), (1.1,18), (1,6), (2.3,66), (1,62), (1,12), (1,7)]
    random_points_idx = [1,6,7]
    model.calculate_model_params([points[idx] for idx in random_points_idx])
    model.fit_points_to_model(points)