import pygame
from pyGrid import pygrid
from agent.human_walker import Human
import random

grid = pygrid.pyGrid(100, 100, 10, 10, 2)
clock = pygame.time.Clock()
done = False

pygame.display.set_caption("people_following")

start_y = random.randrange(grid.height / 8, grid.height / 4)
start_x = random.randrange(grid.width / 8, grid.width - grid.width / 8)
end_y = random.randrange(grid.height - grid.height / 4, grid.height - grid.height / 8)
end_x = random.randrange(grid.width / 8, grid.width - grid.width / 8)

human = Human([start_x, start_y], [end_x, end_y])

while done == False:
    human.generate_path()
    human.move()
    grid.on(human.pos[0], human.pos[1], human.size, (255, 0, 0))
    clock.tick()
