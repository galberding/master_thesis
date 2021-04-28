# '''Visualize path actions'''

import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch, Polygon
import numpy as np
import os
from parseRuns import parsePopulation
import parseRuns as prun
import eval_gen as evalg


def draw_background(ac, ax):
    xyA = ac[0:2]
    xyB = ac[2:4]
    coordsA = "data"
    coordsB = "data"
    background = ConnectionPatch(
        xyA, xyB, coordsA, coordsB,
        # arrowstyle="simple",
        shrinkA=1, shrinkB=1,
        mutation_scale=10, fc="w",
        alpha=1,
        fill=True,
        color="#AAD6FD",
        lw=30,
        zorder=0)
    ax.add_artist(background)


def draw_arrow(ac, ax):
    # plt.arrow(ac[0], ac[1] , ac[2] - ac[0], ac[3] - ac[1],
    #           length_includes_head=True,
    #           head_width=0.05,
    #           head_length=0.002,
    #           lw=3
    #          )
    # print(ac.shape)
    # actions[2:4]-actions[0:2]
    xyA = ac[0:2]
    xyB = ac[2:4]
    coordsA = "data"
    coordsB = "data"
    if np.linalg.norm(xyB-xyA) > .5:

        con = ConnectionPatch(
            xyA, xyB, coordsA, coordsB,
        arrowstyle="simple", shrinkA=10, shrinkB=10,
            mutation_scale=22, fc="w",
            alpha=0.4,
            lw=1,
            zorder=1)

        ax.add_artist(con)
    # plt.show()


def draw_wall(points, ax):
    player = Polygon(points, zorder=1, color="#F5f5f5")
    ax.add_artist(player)


def plot_path(points, walls=[], show=False):
    '''Plot path while given action waypoints in form of:
    AWPs = [[x1,y1,x2,y2],[x2,y2,x3,y3]...] => 4xN
    '''
    ma = points.max()
    mi = points.min()
    offset  = 4

    fig, ax = plt.subplots()
    # fig.dpi = 30
    ax.set_xlim(mi - offset, ma + offset)
    ax.set_ylim(mi - offset, ma + offset)
    for p in points:
        # print(p.shape)
        draw_background(p, ax)
    for p in points:
        # print(p.shape)
        draw_arrow(p, ax)
    color = "#feecdc"
    size = 120
    ax.scatter(points[0,0], points[0,1], color="#000000", zorder=3, s=size)
    ax.scatter(points[-1,2], points[-1,3], color="#Ff0000", zorder=3, marker="X", s=size)
    ax.scatter(points[:, 0], points[:, 1], color=color, s=size, zorder=2)
    ax.scatter(points[4:, 2], points[4:, 3], color=color, s=size, zorder=2)
    for wall in walls:
        draw_wall(wall, ax)
    if show:
        plt.show()


def boustrophedon_movement():
    '''Retrun action waypoints for boustrophedon movement pattern'''
    pos = []

    for i in range(0,10,2):
        # for j in range(0, 11, 10):
        pos.append([i, 0, i, 10]) # up
        pos.append([i, 10, i+1, 10]) # right
        pos.append([i+1, 10, i+1, 0]) # down
        pos.append([i+1, 0, i+2, 0]) # right

    last = pos[-1]
    pos.append([last[2], last[3], last[2], 10])
    points = np.array(pos)
    return points


def spiral_movement():
    x = 1
    y = 12
    points = []
    for i in range(6):
        points.append([x, x-1, x, y])
        points.append([x, y, y, y])
        points.append([y, y, y, x])
        x += 1
        y -= 1
        points.append([y+1, x-1, x, x-1])
    return np.array(points)


def random_movement():
    return np.array([
        [0, 0, 8, 8],
        [8, 8, 2, 5],
        [2, 5, 7, 2]
    ])

def getWalls():
    left = np.array([
        [-2, -1],
        [-2, 11],
        [-0.5, 11],
        [-0.5, -1]
    ])
    down = np.array([
        [-2, -1],
        [12, -1],
        [12, -3],
        [-2, -3]
    ])
    right = np.stack((left[:,0] + 12.5, left[:, 1]), axis=1)
    up = np.stack((down[:,0], down[:,1] + 14), axis=1)
    return [left, right, down, up]

def plot_best_run(path):
    """Path to run with best performance.
    Function will select the last saved pool and plot it
    """
    pools, perfs, run_log = prun.scanRunDir(path)
    # sorted(pools)
    print(pools)
    plot_pool(pools[0])


def plot_pool(path):
    p = prun.parsePopulationPool(path)
    for pl in p:
        actions = pl.actionInfos()
        # actions = actions[~np.all(actions == 0, axis=1)][:-1,:]
        print("Actions:", actions.shape[0])
        if actions.shape[0] > 100:
            plot_path(actions)
            plt.show()


if __name__ == "__main__":
    # np.set_printoptions(threshold=np.inf)
    # path = "/homes/galberding/Projects/Ma/evaluation/global_ws/t1/46_train"
    path = "/home/schorschi/workspace/devel/lib/ros_optimizer/ConvexRegion2/310_pool.actions"
    # pools, perfs, run_log = prun.scanRunDir(path)
    # plot_pool(path)
    # plt.show()
    # points = [
    #     [0, 0, 0, 1],
    #     [0, 1, 1, 1],
    #     [1, 1, 1, 0],
    #     [1, 0, 2, 0],
    #     [2, 0, 0.8, 0.8],
    #     [2, 0, 0.8, 0.6],
    #     [2, 0, 0.8, 0.5],
    #     [2, 0, 0.8, 0.3],
    #     [2, 0, 0.8, 0.2],
    #     [2, 0, 0.8, 0.1],
    # ]



    plot_path(boustrophedon_movement())
    plt.show()
    # plot_path(spiral_movement())
    # plot_path(random_movement())



    # data = []
    # plt.figure(dpi=1200)
    # for i, pp in enumerate(pools):
        # fig, ax = plt.subplots()
        # fig.dpi=300

    # pool = prun.parsePopulationPool(pools[-1])
    # p = pool[0]
    # actions = p.actionInfos()
    # actions = actions[~np.all(actions == 0, axis=1)][:-1,:]

    # aa = actions[2:4]-actions[0:2]
    # plt.quiver(actions[0:2], aa)
    # for ac in p.actionInfos():
    # plot_path(actions)
