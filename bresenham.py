import math
from robotParams import *
from geometry_msgs.msg import Pose2D

# true - obstacle, false - no obstacle
# y - row, x - column

class grid_pos_t():
    def __init__(self, x=None, y=None):
        self.grid_x = x
        self.grid_y = y
    def __str__(self):
        return(' '.join(["y:", str(self.grid_y), "x:", str(self.grid_x)]))
    def __repr__(self):
        return(' '.join(["y:", str(self.grid_y), "x:", str(self.grid_x)]))

def map_Pose2Do_room(pos: Pose2D):
    grid_pos = grid_pos_t()
    grid_pos.grid_x = int((pos.x + CELL_SIZE_METER/2*(MAP_SIZE + 1))/CELL_SIZE_METER)
    grid_pos.grid_y = int((-pos.y + CELL_SIZE_METER/2*(MAP_SIZE + 1))/CELL_SIZE_METER)
    return grid_pos


def map_room_to_position(grid_pos: grid_pos_t):
    pos = Pose2D()
    pos.x = CELL_SIZE_METER*grid_pos.grid_x - MAP_SIZE*CELL_SIZE_METER/2
    pos.y = -CELL_SIZE_METER*grid_pos.grid_y + MAP_SIZE*CELL_SIZE_METER/2
    return pos

def laser_through_tiles(range: float, angle: float, ranger_pos: Pose2D, max_range: float):
    obstacle_pos = Pose2D()
    grid_ranger_pos = grid_pos_t()
    grid_obstacle_pos = grid_pos_t()
    
    # calculate obstacle position basing on ranger measurement
    # if range is over max range no obstacle found
    if range > max_range:
        obstacle_found = False
        obstacle_pos.x = math.cos((ranger_pos.theta + angle)) * max_range + ranger_pos.x
        obstacle_pos.y = math.sin((ranger_pos.theta + angle)) * max_range + ranger_pos.y
    else:
        obstacle_found = True
        obstacle_pos.x = math.cos((ranger_pos.theta + angle)) * range + ranger_pos.x
        obstacle_pos.y = math.sin((ranger_pos.theta + angle)) * range + ranger_pos.y
    
    # convert laser scan to grid and update map by adding unoccupied tiles
    grid_ranger_pos = map_Pose2Do_room(ranger_pos)
    grid_obstacle_pos = map_Pose2Do_room(obstacle_pos)
    empty_tiles = _sensor_update_line(grid_ranger_pos.grid_x, grid_ranger_pos.grid_y,
            grid_obstacle_pos.grid_x, grid_obstacle_pos.grid_y)

    # add obstacle to the map if it was found
    obstacle_tiles = []
    if obstacle_found:
        # if it isnt outside of the map
        if (grid_obstacle_pos.grid_x > 0
                and grid_obstacle_pos.grid_y > 0
                and grid_obstacle_pos.grid_x < MAP_SIZE-1
                and grid_obstacle_pos.grid_y < MAP_SIZE-1
                ):
            obstacle_tiles = [grid_obstacle_pos] #add obstacle
    return empty_tiles, obstacle_tiles
        
def _sensor_update_line(x0: int, y0: int, x1: int, y1: int):
    if abs(y1 - y0) < abs(x1 - x0):
        if x0 < x1:
            return _sensor_bresenham_low(x0, y0, x1, y1, 1)
        else:
            return _sensor_bresenham_low(x0, y0, x1, y1, -1)
    else:
        if y0 < y1:
            return _sensor_bresenham_high(x0, y0, x1, y1, 1)
        else:
            return _sensor_bresenham_high(x0, y0, x1, y1, -1)


def _sensor_bresenham_low(x0: int, y0: int, x1: int, y1: int, sign: int):
    cells_found = []
    dx = x1 - x0
    dy = y1 - y0
    yi = 1
    if sign < 0:
        dx = -dx
        dy = -dy
    if dy < 0:
        yi = -1
        dy = -dy
    if sign < 0:
        yi = -yi
    D = int((dy*2) - dx)
    while not int(x0) == int(x1):
        # only if tile is inside a map (and not on the edge) set tile to unoccupied
        if(x0 > 0 and y0 > 0 and x0 < MAP_SIZE-1 and y0 < MAP_SIZE-1):
            cells_found.append(grid_pos_t(int(x0), int(y0)))
        else:
            return cells_found
        if D > 0:
            y0 += yi
            D += (dy - dx)*2
        else:
            D += dy*2
        x0+=sign
    return cells_found
    

def _sensor_bresenham_high(x0: int, y0: int, x1: int, y1: int, sign: int):
    cells_found = []
    dx = x1 - x0
    dy = y1 - y0
    xi = 1
    if sign < 0:
        dx = -dx
        dy = -dy
    if dx < 0:
        xi = -1
        dx = -dx
    if sign < 0:
        xi = -xi
    D = int((dx*2) - dy)

    while not int(y0) == int(y1):
        # only if tile is inside a map (and not on the edge) set tile to unoccupied
        if(x0 > 0 and y0 > 0 and x0 < MAP_SIZE-1 and y0 < MAP_SIZE-1):
            cells_found.append(grid_pos_t(int(x0), int(y0)))
        else:
            return cells_found
        if D > 0:
            x0 += xi
            D += (dx - dy)*2
        else:
            D += dx*2
        y0+=sign
    return cells_found
