from laserSub import LaserScanListener
import ransac
import logging
import plot
import matplotlib.pyplot as plt
import copy
import numpy as np
import math

__MERGE_TRESHOLD = 0.05

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

for line in lines:
    angle = angle_from_points(line)
    print(angle)

while True: 
    for line_1 in lines:
        line_deleted = False
        for line_2 in lines:
            min_val, first_list = min_dist_between_lines(line_1, line_2)
            if min_val < __MERGE_TRESHOLD:
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
                
    # angle = angle_from_points(line)
    # print(angle)


fig = plot.plot([], [], points, lines)
plt.show()