import numpy as np

import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import pygame
from pyGrid import pygrid


class GridMap(pygrid.pyGrid):

    def __init__(self, x, y, obstacles):

        super(GridMap, self).__init__(x, y)
        self.cells = np.array(self.cells)
        print self.cells.shape
        self._set_obstacles()
        self.draw()


    def _set_obstacles(self):
        left_up = obstacles[0]
        wid_hei = obstacles[1]
        for coord, size in zip(left_up, wid_hei):
            wid = size[1]
            hei = size[0]
            self.cells[coord[1]:coord[1]+hei,coord[0]:coord[0]+wid] = 1


if __name__ == '__main__':
    # (x, y) coordinate of obstacle leftup corner
    obstacle_leftup = [[0, 0], [0,  26], [30,  20], [40, 0]]
    obstacle_hei_wid = [[10, 27], [12, 18], [20, 10], [15, 6]]
    obstacles = (obstacle_leftup, obstacle_hei_wid)
    map = GridMap(50, 50, obstacles)
