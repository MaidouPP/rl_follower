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
        self.x = x
        self.y = y
        self.cells = np.array(self.cells)
        self._obstacles = obstacles
        self.wind = 0.05
        self.n_actions = 4
        self.n_states = self.x * self.y
        self.actions = ((1, 0), (0, 1), (-1, 0), (0, -1))

        # for now, we only deal with deterministic situation
        self.transition_mat = self._get_transition_mat()

        self._set_obstacles()

    def _set_obstacles(self):
        left_up = self._obstacles[0]
        wid_hei = self._obstacles[1]
        for coord, size in zip(left_up, wid_hei):
            wid = size[1]
            hei = size[0]
            self.cells[coord[1]:coord[1] + hei, coord[0]:coord[0] + wid] = 1

    def clear_trajectories(self):
        self.cells[self.cells == 2] = 0

    def _get_transition_mat(self):
        """
        Get transitioning mat with 3 dimension: state, action, state.

        i: State int.
        j: Action int.
        k: State int.
        -> p(s_k | s_i, a_j)

        Since it's deterministic sceneriio, the mat is a 0/1 mat.
        """

        p = np.zeros((self.n_states, self.n_actions, self.n_states))
        for si in xrange(self.n_states):
            pos_si = self.int_to_point(si)
            pos_sj = (0, 0)
            for a in xrange(self.n_actions):
                inc = self.actions[a]
                nei_s = (pos_si[0] + inc[0], pos_si[1] + inc[1])
                if nei_s[0] >= 0 and nei_s[0] < self.height and nei_s[1] >= 0 and nei_s[1] < self.width and self.cells[nei_s[0]][nei_s[1]] != 1:
                    pos_sj = nei_s
                    sj = self.point_to_int(pos_sj)
                    p[si, a, sj] = 1

        return p

    def int_to_point(self, i):
        """
        Convert a state int into the corresponding coordinate.

        i: State int.
        -> (x, y) int tuple.
        """

        return (i % self.x, i // self.x)

    def point_to_int(self, p):
        """
        Convert a coordinate into the corresponding state int.

        p: (x, y) tuple.
        -> State int.
        """

        return p[0] + p[1] * self.x

    def neighbouring(self, i, k):
        """
        Get whether two points neighbour each other. Also returns true if they
        are the same point.

        i: (x, y) int tuple.
        k: (x, y) int tuple.
        -> bool.
        """

        return abs(i[0] - k[0]) + abs(i[1] - k[1]) <= 1


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
            trace.append(grid_map.point_to_int(np.array([x, y])))
            print "x=", x, "y=", y
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            break
        else:
            continue

        grid_map.other(grid_map.int_to_point(trace[-1])[0],
                       grid_map.int_to_point(trace[-1])[1])

        count = 1

        while count <= 40:
            location = grid_map.int_to_point(trace[-1])
            action = pygame.event.poll()

            if action.type == pygame.KEYDOWN:
                if action.key == pygame.K_TAB:
                    break

                if action.key == pygame.K_UP:
                    if grid_map.cells[location[1] - 1][location[0]] <> 0:
                        continue
                    else:
                        state = grid_map.point_to_int((location[0],
                                                       location[1] - 1))
                        trace.append(state)

                elif action.key == pygame.K_DOWN:
                    if grid_map.cells[location[1] + 1][location[0]] <> 0:
                        continue
                    else:
                        state = grid_map.point_to_int((location[0],
                                                       location[1] + 1))
                        trace.append(state)

                if action.key == pygame.K_RIGHT:
                    if grid_map.cells[location[1]][location[0] + 1] <> 0:
                        continue
                    else:
                        state = grid_map.point_to_int((location[0] + 1,
                                                       location[1]))
                        trace.append(state)

                if action.key == pygame.K_LEFT:
                    if grid_map.cells[location[1]][location[0] - 1] <> 0:
                        continue
                    else:
                        state = grid_map.point_to_int((location[0] - 1,
                                                       location[1]))
                        trace.append(state)

                grid_map.other(grid_map.int_to_point(trace[-1])[0],
                               grid_map.int_to_point(trace[-1])[1])

                count += 1

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

    collect_trajectories(grid_map, 'data/trajectories')
