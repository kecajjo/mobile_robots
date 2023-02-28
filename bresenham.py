import math
from robotParams import *
from geometry_msgs.msg import Pose2D

# true - obstacle, false - no obstacle
# y - row, x - column

class grid_pos_t():
    grid_x = None
    grid_y = None

def map_Pose2Do_room(pos: Pose2D):
    grid_pos = grid_pos_t()
    grid_pos.grid_x = (pos.x + CELL_SIZE/2*(ROOM_SIZE + 1))/CELL_SIZE
    grid_pos.grid_y = (pos.y + CELL_SIZE/2*(ROOM_SIZE + 1))/CELL_SIZE
    return grid_pos


def map_room_to_position(grid_pos: grid_pos_t):
    pos = Pose2D()
    pos.x = CELL_SIZE*grid_pos.grid_x - ROOM_SIZE*CELL_SIZE/2
    pos.y = CELL_SIZE*grid_pos.grid_y - ROOM_SIZE*CELL_SIZE/2
    return pos

def range_update_map(range: float, angle: float, ranger_pos: Pose2D, max_range: float):
    obstacle_pos = Pose2D()
    grid_ranger_pos = grid_pos_t()
    grid_obstacle_pos = grid_pos_t()
    
    # calculate obstacle position basing on ranger measurement
    # if range is over max range no obstacle found
    if range > max_range:
        obstacle_found = False
        obstacle_pos.x = math.cos((ranger_pos.theta + angle)*math.pi/180) * max_range + ranger_pos.x
        obstacle_pos.y = math.sin((ranger_pos.theta + angle)*math.pi/180) * max_range + ranger_pos.y
    else:
        obstacle_found = True
        obstacle_pos.x = math.cos((ranger_pos.theta + angle)*math.pi/180) * range + ranger_pos.x
        obstacle_pos.y = math.sin((ranger_pos.theta + angle)*math.pi/180) * range + ranger_pos.y
    

    # convert laser scan to grid and update map by adding unoccupied tiles
    grid_ranger_pos = map_Pose2Do_room(ranger_pos)
    grid_obstacle_pos = map_Pose2Do_room(obstacle_pos)
    empty_tiles = sensor_update_line(grid_ranger_pos.grid_x, grid_ranger_pos.grid_y,
            grid_obstacle_pos.grid_x, grid_obstacle_pos.grid_y)

    # add obstacle to the map if it was found
    obstacle_tiles = []
    if obstacle_found:
        # if it isnt outside of the map
        if (grid_obstacle_pos.grid_x > 0
                and grid_obstacle_pos.grid_y > 0
                and grid_obstacle_pos.grid_x < ROOM_SIZE-1
                and grid_obstacle_pos.grid_y < ROOM_SIZE-1
                ):
            obstacle_tiles = [(grid_obstacle_pos.grid_y, grid_obstacle_pos.grid_x, True)] #add obstacle
    return empty_tiles, obstacle_tiles
        
def sensor_update_line(x0: int, y0: int, x1: int, y1: int):
    if abs(y1 - y0) < abs(x1 - x0):
        if x0 < x1:
            return sensor_bresenham_low(x0, y0, x1, y1, 1)
        else:
            return sensor_bresenham_low(x0, y0, x1, y1, -1)
    else:
        if y0 < y1:
            return sensor_bresenham_high(x0, y0, x1, y1, 1)
        else:
            return sensor_bresenham_high(x0, y0, x1, y1, -1)


def sensor_bresenham_low(x0: int, y0: int, x1: int, y1: int, sign: int):
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
    D = (dy*2) - dx
    while x0!=x1:
        # only if tile is inside a map (and not on the edge) set tile to unoccupied
        if(x0 > 0 and y0 > 0 and x0 < ROOM_SIZE-1 and y0 < ROOM_SIZE-1):
            cells_found.append(grid_pos_t(x0, y0))
        if D > 0:
            y0 += yi
            D += (dy - dx)*2
        else:
            D += dy*2
        x0+=sign
    return cells_found
    

def sensor_bresenham_high(x0: int, y0: int, x1: int, y1: int, sign: int):
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
    D = (dx*2) - dy
    while y0!=y1:
        # only if tile is inside a map (and not on the edge) set tile to unoccupied
        if(x0 > 0 and y0 > 0 and x0 < ROOM_SIZE-1 and y0 < ROOM_SIZE-1):
            cells_found.append(grid_pos_t(x0, y0))
        if D > 0:
            x0 += xi
            D += (dx - dy)*2
        else:
            D += dx*2
        y0+=sign
    return cells_found
