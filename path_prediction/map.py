import numpy as np
import pickle

import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import pygame
from pyGrid import pygrid


class GridMap(pygrid.pyGrid):

    def __init__(self, x, y, obstacles):

        super(GridMap, self).__init__(x, y)
        self.cells = np.array(self.cells)
        self._set_obstacles()

    def _set_obstacles(self):
        left_up = obstacles[0]
        wid_hei = obstacles[1]
        for coord, size in zip(left_up, wid_hei):
            wid = size[1]
            hei = size[0]
            self.cells[coord[1]:coord[1] + hei, coord[0]:coord[0] + wid] = 1

    def clear_trajectories(self):
        self.cells[self.cells==2] = 0


def collect_trajectories(grid_map, file):
    event = pygame.event.poll()
    grid_map.clear()
    grid_map.draw()

    pygame.display.set_caption("map")
    pygame.mouse.set_visible(1)
    trajectories = []

    while True:
        event = pygame.event.poll()
        trace = []

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = pygame.mouse.get_pos()
            x = pos[0] / (grid_map.cell_width + grid_map.border_weight)
            y = pos[1] / (grid_map.cell_height + grid_map.border_weight)
            trace.append(np.array([x, y]))
            print "x=", x, "y=", y
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            break
        else:
            continue

        grid_map.other(trace[-1][0], trace[-1][1])

        while True:
            location = trace[-1]
            action = pygame.event.poll()

            if action.type == pygame.KEYDOWN:
                if action.key == pygame.K_TAB:
                    break

                if action.key == pygame.K_UP:
                    if grid_map.cells[location[1]-1][location[0]] <> 0:
                        continue
                    else:
                        trace.append(np.array(location + [0, -1]))

                elif action.key == pygame.K_DOWN:
                    if grid_map.cells[location[1] + 1][location[0]] <> 0:
                        continue
                    else:
                        trace.append(np.array(location + [0, 1]))

                if action.key == pygame.K_RIGHT:
                    if grid_map.cells[location[1]][location[0] + 1] <> 0:
                        continue
                    else:
                        trace.append(np.array(location + [1, 0]))

                if action.key == pygame.K_LEFT:
                    if grid_map.cells[location[1]][location[0] - 1] <> 0:
                        continue
                    else:
                        trace.append(np.array(location + [-1, 0]))

                grid_map.other(trace[-1][0], trace[-1][1])

        trajectories.append(trace)
        grid_map.clear()
        grid_map.clear_trajectories()
        grid_map.draw()

    trajectories = np.array(trajectories)

    with open(file, 'w') as f:
        pickle.dump(trajectories, f)


if __name__ == '__main__':
    # (x, y) coordinate of obstacle leftup corner
    obstacle_leftup = [[0, 0], [0,  26], [30,  20], [40, 0]]
    obstacle_hei_wid = [[10, 27], [12, 18], [20, 10], [15, 6]]
    obstacles = (obstacle_leftup, obstacle_hei_wid)
    grid_map = GridMap(50, 50, obstacles)

    collect_trajectories(grid_map, 'trajectories')
