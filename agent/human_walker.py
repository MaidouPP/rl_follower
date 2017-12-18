import pygame
import numpy as np
import random
import math

class Human():
    """
    This class implements a basic random human walker from
    a starting point to a goal with shortest path.
    """
    def __init__(self, start, end, path=None, size=1):
        self._start = start
        self._end = end
        self._path = path
        self._idx = 0
        self.size = size
        self.pos = start

    def clear_path(self):
        self._path = None

    def generate_path(self):
        if self._path is None:
            self._path = []
            x = self._end[0] - self._start[0]
            y = self._end[1] - self._start[1]

            len_of_path = max(abs(x), abs(y))
            xs = []
            ys = []
            unit_x = x / abs(x)
            unit_y = y / abs(y)

            if abs(x) >= abs(y):
                xs = [[unit_x]] * len_of_path
                ys = [[unit_y]] * abs(y)
                ys += [[0]] * (len_of_path - abs(y))
            else:
                ys = [[unit_y]] * len_of_path
                xs = [[unit_x]] * abs(x)
                xs += [[0]] * (len_of_path - abs(x))

            random.shuffle(xs)
            random.shuffle(ys)
            self._path = np.array([xs, ys]).squeeze()
            print self._path.shape

    def move(self):
        self.pos[0] += self._path[0][self._idx]
        self.pos[1] += self._path[1][self._idx]
        self._idx += 1
        if self._idx >= self._path.shape[1]:
            return False
        else:
            return True

if __name__ == '__main__':
    test = Human([0, 0], [3, 10])
    test.generate_path()
    test.move()
