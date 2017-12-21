import pygame
from pyGrid import pygrid
from agent.human_walker import Human
import random
import numpy as np
from agent.agent import RL, SarsaAgent
import pickle

grid = pygrid.pyGrid(100, 100, 10, 10, 2)
clock = pygame.time.Clock()
done = False

pygame.display.set_caption("people_following")

action_space = [0, 1, 2, 3, 4, 5, 6, 7, 8]
max_distance = 15
# state_space = [(x, y) for x in range(-max_distance, max_distance + 1)
               # for y in range(-max_distance, max_distance + 1)]
# state_space = np.array(state_space)
state_space = [2 * max_distance + 1, 2 * max_distance + 1]
comfortable_radius = 3

robot = SarsaAgent(action_space,
                   state_space,
                   learning_rate=0.01,
                   reward_decay=0.9,
                   trace_decay=0.9,
                   epsilon=0.9,
                   max_distance=max_distance,
                   last_state=np.array([max_distance, max_distance-2]),
                   last_robot_position=np.array([0, 0]),
                   last_human_position=np.array([0, 0]),
                   comfortable_radius=comfortable_radius)


def step(act, pos):
    if act == 0:
        return np.array([pos[0], pos[1]])
    elif act == 1:
        return np.array([pos[0] - 1, pos[1] - 1])
    elif act == 2:
        return np.array([pos[0], pos[1] - 1])
    elif act == 3:
        return np.array([pos[0] + 1, pos[1] - 1])
    elif act == 4:
        return np.array([pos[0] - 1, pos[1]])
    elif act == 5:
        return np.array([pos[0] + 1, pos[1]])
    elif act == 6:
        return np.array([pos[0] - 1, pos[1] + 1])
    elif act == 7:
        return np.array([pos[0], pos[1] + 1])
    elif act == 8:
        return np.array([pos[0] + 1, pos[1] + 1])


def train():
    for i in xrange(100000):
        start_y = random.randrange(grid.height / 8, grid.height / 4)
        start_x = random.randrange(grid.width / 8, grid.width - grid.width / 8)
        end_y = random.randrange(
            grid.height - grid.height / 4, grid.height - grid.height / 8)
        end_x = random.randrange(grid.width / 8, grid.width - grid.width / 8)

        robot_position = np.array([start_x, start_y - 2])
        robot.reinit(last_robot_position=robot_position,
                     last_human_position=np.array([start_x, start_y]))

        human = Human([start_x, start_y], [end_x, end_y])
        human.generate_path()
        done = False

        while done == False:
            # human.generate_path()
            if human.move() is False:
                done = True
            # grid.on(human.pos[0], human.pos[1], 1, (255, 0, 0))

            robot_pos = step(robot.last_action, robot.last_robot_position)
            robot.learn(robot_pos, np.array(human.pos))

            # grid.on(robot_pos[0], robot_pos[1], 1, (0, 255, 0))

            # clock.tick(1000)

        if i % 100 == 0:
            print "=== Round :", i, " ==="
        grid.clear()

def action_for_test(q, robot_position, human_position):
    new_state = [human_position[0] - robot_position[0],
                 human_position[1] - robot_position[1]]
    new_state[0] = max(min(new_state[0], max_distance), -max_distance)
    new_state[1] = max(min(new_state[1], max_distance), -max_distance)
    new_state[0] += max_distance
    new_state[1] += max_distance
    act = np.argmax(q[new_state[0], new_state[1], :])
    return act


def test():
    f = open('q_result', 'r')
    q = pickle.load(f)
    f.close()

    for i in xrange(10):
        start_y = random.randrange(grid.height / 8, grid.height / 4)
        start_x = random.randrange(grid.width / 8, grid.width - grid.width / 8)
        end_y = random.randrange(
            grid.height - grid.height / 4, grid.height - grid.height / 8)
        end_x = random.randrange(grid.width / 8, grid.width - grid.width / 8)

        robot_pos = np.array([start_x, start_y - 2])
        # robot.reinit(last_robot_position=robot_position,
                     # last_human_position=np.array([start_x, start_y]))

        human = Human([start_x, start_y], [end_x, end_y])
        human.generate_path()
        done = False

        while done == False:
            # human.generate_path()
            if human.move() is False:
                done = True
            grid.on(human.pos[0], human.pos[1], 1, (255, 0, 0))

            act = action_for_test(q, robot_pos, human.pos)
            robot_pos = step(act, robot_pos)

            grid.on(robot_pos[0], robot_pos[1], 1, (0, 255, 0))

            clock.tick(10)

        grid.clear()


if __name__ == '__main__':
    # train()
    # f = open("q_result", 'w')
    # pickle.dump(robot.q_table, f)
    # f.close()
    test()
