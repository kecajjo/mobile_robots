from bresenham import grid_pos_t
import numpy as np
import copy

def find_path(map: np.array, destination: grid_pos_t, start_pose: grid_pos_t, max_iterations = 1000):
    map = update_map(map, destination, start_pose, max_iterations)
    return get_path_from_map(map, destination, start_pose), map

def update_map(map: np.array, destination: grid_pos_t, start_pose: grid_pos_t, max_iterations):
    map[destination.grid_y][destination.grid_x] = 1
    for iter in range(1, max_iterations):
        for i in range(1, np.size(map, axis=0)-1):
            for j in range(1, np.size(map, axis=1)-1):
                if map[i][j] == 0:
                    if map[i-1][j] == iter or map[i+1][j] == iter or map[i][j-1] == iter or map[i][j+1] == iter:
                        map[i][j] = iter+1
        if map[start_pose.grid_y][start_pose.grid_x] != 0:
            break
    else:
        raise Exception("path not found")
    return map

def get_path_from_map(map: np.array, destination: grid_pos_t, start_pose: grid_pos_t):
    x = start_pose.grid_x
    y = start_pose.grid_y
    i = map[y][x]
    path = [start_pose]
    while x != destination.grid_x or y != destination.grid_y:
        i-=1
        if map[y][x+1] == i:
            path.append(grid_pos_t(x+1,y))
            x += 1
        elif map[y][x-1] == i:
            path.append(grid_pos_t(x-1,y))
            x -= 1
        elif map[y-1][x] == i:
            path.append(grid_pos_t(x,y-1))
            y -=1
        elif map[y+1][x] == i:
            path.append(grid_pos_t(x,y+1))
            y += 1
        else:
            raise Exception("path is broken")
    return path
    
def is_obstacle(obstacle):
    if np.isinf(obstacle):
        return True
    return False
