from laserSub import LaserScanListener
from geometry_msgs.msg import Point
import ransac
import logging
import plot
import matplotlib.pyplot as plt
import copy
import numpy as np
import math

__MERGE_DIST_TRESHOLD = 0.05
__MERGE_ANG_TRESHOLD_DEG = 20
__ZERO_TRESHOLD = 0.05

def angle_from_points(points):
    list_x = [point.x for point in points]
    list_y = [point.y for point in points]
    result = np.polyfit(list_x, list_y, 1)
    return np.arctan(result[0])*180/3.14

def dist_between_points(point1, point2):
    dist = math.hypot(point2.x - point1.x, point2.y - point1.y)
    return dist

def min_dist_between_lines(line_1, line_2):
    # if index is 0 then line 1 is first
    # if index is 1 then line 2 is first
    dist = []
    dist.append(dist_between_points(line_1[-1],line_2[0]))
    dist.append(dist_between_points(line_1[0],line_2[-1]))

    min_val = min(dist)
    min_index = dist.index(min_val)
    return min_val, min_index

def merge_lines(line_1, line_2):
    line_1.extend(line_2)
    return line_1

def point_between_2_points(p1, p2):
    x = (p1.x + p2.x)/2
    y = (p1.y + p2.y)/2
    return x, y

def remove_duplicate_points(points, precision):
    unique_points = []
    for point in points:
        x, y = point
        for unique_point in unique_points:
            ux, uy = unique_point
            if round(x, precision) == round(ux, precision) and round(y, precision) == round(uy, precision):
                break
        else:
            unique_points.append(point)
    return unique_points


def merge_parallel_lines(lines):
    while True:
        for line_1 in lines:
            line_deleted = False
            for line_2 in lines:
                if abs(angle_from_points(line_1) - angle_from_points(line_2)) <= __MERGE_ANG_TRESHOLD_DEG:
                    min_val, first_list = min_dist_between_lines(line_1, line_2)
                    if min_val < __MERGE_DIST_TRESHOLD:
                        if first_list == 0:
                            merged_line = merge_lines(line_1, line_2)
                        elif first_list == 1:
                            merged_line = merge_lines(line_2, line_1)
                        else:
                            raise Exception
                        lines.remove(line_1)
                        lines.remove(line_2)
                        lines.append(merged_line)
                        line_deleted = True
                        break
            if line_deleted:
                break
        else:
            break
    return lines

def find_corners(lines):
    corners = []
    for line_1 in lines:
        for line_2 in lines:
            min_val, first_list = min_dist_between_lines(line_1, line_2)
            if min_val < __MERGE_DIST_TRESHOLD:
                if first_list == 0:
                    corners.append(point_between_2_points(line_1[-1], line_2[0]))
                elif first_list == 1:
                    corners.append(point_between_2_points(line_2[-1], line_1[0]))
                else:
                    raise Exception
    return corners

def make_zero_point_from_point(lines, translate):
    new_lines = []
    for line in lines:
        new_line = []
        for point in line:
            new_line.append(Point(point.x - translate[0], point.y - translate[1], 0))
        new_lines.append(new_line)
    return new_lines

def rotate_coordinate_system(lines, angle):
    new_lines = []
    for line in lines:
        new_line = []
        for point in line:
            xr = (point.x * math.cos(angle)) - (point.y * math.sin(angle))
            yr = (point.x * math.sin(angle)) + (point.y * math.cos(angle))
            new_line.append(Point(xr, yr, 0))
        new_lines.append(new_line)
    return new_lines

def rotate_and_check_coord(lines):
    for i in range (5000):
        counter = 0
        lines = rotate_coordinate_system(lines, 0.05)
        for line in lines:
            if dist_between_points(line[0], Point()) < __ZERO_TRESHOLD:
                if (abs(line[-1].x) < __ZERO_TRESHOLD and line[-1].y > 0) or (
                    abs(line[-1].y) < __ZERO_TRESHOLD and line[-1].x > 0):
                    counter+=1
            if dist_between_points(line[-1], Point()) < __ZERO_TRESHOLD:
                if (abs(line[0].x) < __ZERO_TRESHOLD and line[0].y > 0) or (
                    abs(line[0].y) < __ZERO_TRESHOLD and line[0].x > 0):
                    counter+=1
        if counter >= 2:
            return lines



logging.basicConfig(level=logging.INFO)
sub = LaserScanListener()
sub.load_json_scan()
lst = sub.get_point_list()
points = copy.deepcopy(lst)
model = ransac.Line()
lines = []
line_not_found_times = 0
while line_not_found_times < 5:
    new_line = ransac.extract_line(lst, 10, model)
    if new_line is None:
        line_not_found_times += 1
    else:
        lines += new_line

lines = merge_parallel_lines(lines)
corners = find_corners(lines)
corners = remove_duplicate_points(corners, 5)

fig = plot.plot([], [], points, lines, corners)
plt.show()

translate = corners[-1]
# current_robot_pos is minus translate
lines = make_zero_point_from_point(lines, translate)
# lines = rotate_coordinate_system(lines, 3.14/2)
lines = rotate_and_check_coord(lines)
    # angle = angle_from_points(line)
    # print(angle)


fig = plot.plot([], [], [], lines, [[0.0,0.0]])
plt.show()