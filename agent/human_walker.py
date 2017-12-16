import pygame
import numpy as np
import random

class Human():
    """
    This class implements a basic random human walker from
    a starting point to a goal with shortest path.
    """
    def __init__(self, start, end, path=None, size=3):
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
            if y > x:
                self._path = [[1, 1]] * x
                self._path += [[0, 1]] * (y - x)
            else:
                self._path = [[1, 1]] * y
                self._path += [[1, 0]] * (x - y)

        random.shuffle(self._path)
        print self._path

    def move(self):
        self.pos[0] += self._path[self._idx][0]
        self.pos[1] += self._path[self._idx][1]
        self._idx += 1
        if self._idx >= len(self._path):
            return False
        else:
            return True

if __name__ == '__main__':
    test = Human([0, 0], [3, 10])
    test.generate_path()
    test.move()
