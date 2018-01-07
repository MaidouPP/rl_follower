import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy.ndimage.filters import gaussian_filter
from scipy import signal


import maxent
from map import GridMap


def load_trajectories(file):
    trajectories = []
    with open(file, 'r') as f:
        trajectories = pickle.load(f)

    return trajectories

def create_features(grid_map):
    map_values = grid_map.cells
    map_values.astype(float)
    # feature 1: obstacle or not
    feature_1 = map_values
    feature_1 = feature_1.reshape((-1, 1))

    # feature 2: whether obstacle is around (8 adjacent grids)
    kernel = np.array([[1, 1, 1],
                       [1, 1, 1],
                       [1, 1, 1]])
    feature_2 = signal.convolve2d(map_values, kernel,
                                  boundary='symm', mode='same')
    feature_2[feature_2 > 0] = 1
    feature_2 = feature_2.reshape((-1, 1))

    # feature 3: blurred obstacle, sigma = 1
    feature_3 = gaussian_filter(map_values, sigma=0.5)
    feature_3 = feature_3.reshape((-1, 1))

    # feature 4: blurred obstacle, sigma = 2
    feature_4 = gaussian_filter(map_values, sigma=1)
    feature_4 = feature_4.reshape((-1, 1))

    # feature 5: blurred obstacle, sigma = 3
    feature_5 = gaussian_filter(map_values, sigma=2)
    feature_5 = feature_5.reshape((-1, 1))

    # feature 6: blurred obstacle, sigma = 4
    feature_6 = gaussian_filter(map_values, sigma=4)
    feature_6 = feature_6.reshape((-1, 1))

    # concatenate the features
    feature = [feature_1, feature_2, feature_3,
               feature_4, feature_5, feature_6]

    feature = np.concatenate((feature_1, feature_2, feature_3,
                              feature_4, feature_5, feature_6), axis=1)

    return feature


def model_world(world):
    create_features(world)


if __name__ == '__main__':
    obstacle_leftup = [[0, 0], [0, 26], [30,  20], [40, 0]]
    obstacle_hei_wid = [[10, 27], [12, 18], [20, 10], [15, 6]]
    obstacles = (obstacle_leftup, obstacle_hei_wid)
    grid_map = GridMap(50, 50, obstacles)

    traj = load_trajectories('data/trajectories')
    print traj
    features = create_features(grid_map)
    r = maxent.irl(features, 4, 0.99, grid_map.transition_probability,
                   traj, 20, 0.05)

    f = open('data/reward/grid_world_reward', 'w')
    pickle.dump(r, f)
    f.close()

    plt.pcolor(r.reshape((grid_map.x, grid_map.y)))
    plt.colorbar()
    plt.title("Recorvered Reward")
    plt.show()
