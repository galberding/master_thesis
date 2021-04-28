import seaborn as sns
# from votes import long as df
import matplotlib.pyplot as plt
import numpy as np
import os
from parseRuns import parsePopulation
import parseRuns as prun
import eval_gen as evalg


def plotMeansToGenMeans(path, name):
    pop = parsePopulation(path, name)
    # ll = pop[0].to_array()
    means = []
    for i in range(len(pop)):
        ll = pop[i].to_array()
        mean = ll.mean(axis=0)
        # print(mean)
        means.append(mean)
        # plt.scatter(mean[0], mean[1])
    meansArr = np.array(means)
    pop_mean = meansArr.mean(axis=0)
    for m in means:
        draw = np.stack((m, pop_mean), axis=0)
        print(m, "-->", np.linalg.norm(m-pop_mean))
        plt.plot(draw[:, 0], draw[:, 1])
    plt.title(name)
    plt.show()


def plotMeansDist(path, name):
    pop = parsePopulation(path, name)
    # ll = pop[0].to_array()
    means = []
    for i in range(len(pop)):
        ll = pop[i].to_array()
        mean = ll.mean(axis=0)
        print(mean)
        means.append(mean)
        plt.scatter(mean[0], mean[1])
    # meansArr = np.array(means)
    # pop_mean = meansArr.mean(axis=0)
    # for m in means:
    #     draw = np.stack((m,pop_mean), axis=0)
    #     print(m, "-->", np.linalg.norm(m-pop_mean))
    #     plt.plot(draw[:,0], draw[:,1])
    plt.title(name)
    plt.show()


def visAllIterations():
    '''Entrypoint for visualizing means of a population'''
    path = "/homes/galberding/catkin_ws/devel/lib/ros_optimizer/elitistSel/"
    for i in range(0, 10000, 1000):
        plotMeansDist(path, str(i) + "_pool.actions")
        plotMeansToGenMeans(path, str(i) + "_pool.actions")


def visRunDiv():
    print(sns.load_dataset("tips"))


def calDistMatrix(pop):
    maxLen = prun.getMaxPoolActionLen(pop)
    dists = []
    for i in range(len(pop)):
        row = []
        for j in range(len(pop)):
            row.append(np.linalg.norm(pop[j].describeActions(maxLen)
                       - pop[i].describeActions(maxLen)))
        dists.append(row)
    return np.stack(dists)


def visDistanceMatrix(pop, save=None):
    D = calDistMatrix(pop)
    plt.imshow(D, cmap="gray")
    if save:
        plt.savefig(save + ".png")
    else:
        plt.show()


def visDistanceMatrixIndividualAction(pop, save=None, idx=None):
    D = prun.calActionMinDist(pop)
    plt.imshow(D, cmap="gray")
    if save:
        plt.savefig(os.path.join(save, idx+ ".png"))
        np.savetxt(os.path.join(save, idx + ".csv"), D, delimiter=',')
    else:
        plt.show()


def visBoxPlot(run_path, res_path):
    div_name, data, labels = evalg.collectDistanceInformation(res_path)
    rlog = prun.parseRunLogFile(run_path)
    plt.subplot(2, 1, 2)
    idx =  []
    real_idx = []
    for i in range(len(labels)):
        if labels[i] % 1000:
            idx.append(i)
            real_idx.append(labels[i])
            run = data[i][np.triu_indices_from(data[0], 1)]
            plt.boxplot(run, positions=[labels[i]])
    plt.subplot(2, 1, 1)
    plt.plot(real_idx, rlog[real_idx][:, 2])

    plt.xticks(rotation=45)
    plt.show()


if __name__ == "__main__":
    # visRunDiv()
    import numpy as np
    import matplotlib.pyplot as plt

    # Fixing random state for reproducibility
    np.random.seed(19680801)

    # fake up some data
    spread = np.random.rand(50) * 100
    print(spread.shape)
    center = np.ones(25) * 50
    flier_high = np.random.rand(10) * 100 + 100
    flier_low = np.random.rand(10) * -100
    data = np.concatenate((spread, center, flier_high, flier_low))
    print(data.shape)
