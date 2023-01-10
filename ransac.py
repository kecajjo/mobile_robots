#!/usr/bin/env python

import random

_MAX_ITER = 5000
_MAX_ONE_ITERATION_PER_X_POINTS = 50
_MIN_POINTS_TO_CALC_FACTOR = 1.5
_RANDOM_SAMPLES = 5

def extract_line(points: list, threshold: int):
    lines = []
    iterations = min(_MAX_ITER, len(points)/_MAX_ONE_ITERATION_PER_X_POINTS)
    for i in range(iterations):
        if len(points) < threshold*_MIN_POINTS_TO_CALC_FACTOR:
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
            raise Exception("left_index under 0")
        
        num_samples = _RANDOM_SAMPLES
        if threshold < _RANDOM_SAMPLES:
            _RANDOM_SAMPLES = threshold/2
        random_points_idx = random.sample(range(left_index, right_index), num_samples)
        _calculate_model_params([points[idx] for idx in random_points_idx])


