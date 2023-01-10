from laserSub import LaserScanListener
import ransac
import logging
import plot
import matplotlib.pyplot as plt
import copy

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
fig = plot.plot([], [], points, lines)
plt.show()