import numpy as np


class RL(object):
    """
    This class implements a basic class of reinforcement
    learning framework which is supposed to be inherited
    by other RL-related agent classes.
    """

    def __init__(self,
                 action_space,
                 state_space,
                 learning_rate=0.01,
                 reward_decay=0.9,
                 epsilon=0.9):
        self.actions = action_space  # a list
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = epsilon

        self.q_table = np.zeros(
            (state_space[0], state_space[1],
             len(action_space)))

        self.state_space = state_space

    def greedy_action_choose(self, state):
        if np.random.rand() < self.epsilon:
            # choose best action
            action = self.actions[np.argmax(
                self.q_table[state[0], state[1], :])]
        else:
            # choose random action
            action = np.random.choice(self.actions)
        return action


class SarsaAgent(RL):
    """
    This class implements a reinforcement learning agent that
    adopts sarsa learning method to follow a designated object.
    """

    def __init__(self,
                 action_space,
                 state_space,
                 last_state,
                 last_robot_position,
                 last_human_position,
                 comfortable_radius,
                 learning_rate=0.01,
                 reward_decay=0.9,
                 trace_decay=0.9,
                 epsilon=0.9,
                 max_distance=15
):

        super(SarsaAgent, self).__init__(action_space,
                                         state_space,
                                         learning_rate,
                                         reward_decay,
                                         epsilon)

        self.lambda_ = trace_decay
        self.eligibility_trace = self.q_table.copy()
        self.last_state = last_state
        self.last_robot_position = last_robot_position
        self.last_human_position = last_human_position
        self.last_action = 0
        # self.action = None
        self.comfortable_radius = comfortable_radius
        self.max_distance = max_distance

    def reinit(self,
               last_state=np.array([0, 0]),
               last_robot_position=np.array([0, 0]),
               last_human_position=np.array([0, 0])):
        self.eligibility = np.zeros((self.state_space[0], self.state_space[1],
                                     len(self.actions)))
        self.last_state = np.array([self.max_distance, self.max_distance-2])
        self.last_robot_position = last_robot_position
        self.last_human_position = last_human_position

    def state_mapping(self, robot_position, human_position):
        new_state = [human_position[0] - robot_position[0],
                     human_position[1] - robot_position[1]]
        new_state[0] = max(min(new_state[0], self.max_distance), -self.max_distance)
        new_state[1] = max(min(new_state[1], self.max_distance), -self.max_distance)
        new_state[0] += self.max_distance
        new_state[1] += self.max_distance
        return new_state

    def calculate_reward(self, human_position):
        """
        This method calculates the reward of last state.
        """
        reward = 0
        last_distance = np.sqrt(np.dot(self.last_human_position - self.last_robot_position, self.last_human_position - self.last_robot_position))
        new_distance = np.sqrt(
            np.dot(human_position - self.last_robot_position, human_position - self.last_robot_position))

        if last_distance > new_distance:
            reward -= 20
        if abs(self.last_state[0] - self.max_distance) <= self.comfortable_radius and abs(self.last_state[1] - self.max_distance) <= self.comfortable_radius:
            reward += 1
        else:
            reward -= max(abs(self.last_state[0] - self.max_distance) - self.comfortable_radius, 0) + max(
                abs(self.last_state[1] - self.max_distance) - self.comfortable_radius, 0)

        return reward

    def learn(self, robot_position, human_position):
        new_state = self.state_mapping(robot_position, human_position)
        new_action = self.greedy_action_choose(new_state)

        reward = self.calculate_reward(human_position)
        error = reward + self.gamma * self.q_table[new_state[0], new_state[1], new_action] - self.q_table[self.last_state[0], self.last_state[1], self.last_action]

        # update q-table and eligibility trace
        self.eligibility_trace[self.last_state[0], self.last_state[1], self.last_action] += 1
        self.q_table += self.lr * error * self.eligibility_trace

        # decay eligibility trace of update
        self.eligibility_trace *= self.gamma * self.lambda_

        self.last_action = new_action
        self.last_state = new_state
        self.last_robot_position = robot_position
        self.last_human_position = human_position
