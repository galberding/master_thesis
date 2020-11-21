#!/root/miniconda3/bin/python

import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter



def fitness(x,y):
    return x - y

def main():
    fig = plt.figure()
    ax = fig.gca(projection='3d')


    occ = np.linspace(0,1,100)
    time = np.linspace(0,1,100)
    X, Y = np.meshgrid(occ, time)
    Z = fitness(X,Y)

    print(occ)

    surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

    # Customize the z axis.
    ax.set_zlim(-1.01, 1.01)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter("%.02f"))

    # Add a color bar which maps values to colors.
    fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()




main()
