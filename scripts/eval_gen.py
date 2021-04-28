'''Contains all evaluation functions that can be used by the visualization module'''

# import parseRuns as prun
import os
import numpy as np
import re
from re import search


def calDistMat(pool, fun):
    D = np.zeros((len(pool), len(pool)))
    for row in range(len(pool)):
        for col in range(row, len(pool)):
            val = fun(
                pool[row].actionInfos(),
                pool[col].actionInfos())
            D[row][col] = D[col][row] = val
    return D


def sequenceDirectComparison(a, b):
    A = a
    B = b
    # Equalize length of both genomes
    diff = 0
    if A.shape[0] != B.shape[0]:
        if A.shape[0] > B.shape[0]:
            A = b
            B = a
        diff = abs(B.shape[0] - A.shape[0])
        # print(diff)
        B = B[:-diff]
        # print(B.shape, A.shape)

        # Set distance and direction vector to 0
        # print(B[-2][0])
        # TODO: Not exactly correct because last position is here overwritten
        # lastRow = [[A[-2][0], A[-2][1], 0, 0]]
        # offset = np.repeat(lastRow,
        #                    diff, axis=0)
        # A = np.vstack((A, offset))
    A_diff = A-B
    A_norms = np.linalg.norm(A_diff[:,0:2], axis=1) + np.linalg.norm(A_diff[:,2:-1], axis=1)
        # + A_diff[:, 2] + A_diff[:, 3]
    # mean = A_norms.mean()
    # print(mean)
    return ((A_norms).sum() / A.shape[0]  + diff)**2


def sequenceContainmentComparison(a, b):
    # A = (  S  |    V    |  D )
    # A = (x, y, x1, y1)
    # TODO: Remove all actions with dist == 0
    minSum = 0
    A = a
    B = b
    # A_diff
    # r[~np.all(r == 0, axis=1)]
    A = A[~np.all(A == 0, axis=1)]
    B = B[~np.all(B == 0, axis=1)]
    if A.shape[0] > B.shape[0]:
        A = b
        B = a

    # Remove zero distance entrys
    # A = A[A[:,0] > 0 and A[:,1] > 0 and A[:,2] > 0 and A[:,3] > 0]
    # for i in range(A.shape[1]):
    #     A = A[A[:, i] > 0]
    #     B = B[B[:, i] > 0]

    # B = B[B[:,-1] > 0]

    for b in B:
        # print(Diff_S)
        A_diff = A - b
        # A_diff = A_diff[~np.all(A_diff == 0, axis=1)]
        # for i in range(A_diff.shape[1]):
            # A_diff = A_diff[A_diff[:]]

        Diff1 = np.sum(np.absolute(A - b), axis=1)
        # Diff1 = np.linalg.norm(A - b, axis=1)
        # Diff2 = np.linalg.norm(A[:, 2:-1] - b[2:-1], axis=1)
        # Diff2 = np.linalg.norm(A[:, 2:-1] - b[2:-1], axis=1)
        minSum += np.min(Diff1 )


    return minSum / A.shape[0]


def meanWPComparison(a, b):
    # print(a[:, 0:2].mean(axis=0).shape)
    return np.linalg.norm(a[:, 0:2].mean(axis=0)
                          - b[:, 0:2].mean(axis=0))


def meanActionVComparison(a, b):
    # return np.linalg.norm(a[:, 2:-1].mean(axis=0)
    #                       - b[:, 2:-1].mean(axis=0))
    A = a
    B = b
    # Equalize length of both genomes
    if A.shape[0] != B.shape[0]:
        diff = 0
        if A.shape[0] > B.shape[0]:
            A = b
            B = a
        diff = abs(B.shape[0] - A.shape[0])
        # print(diff)
        B = B[:-diff]
        # print(B.shape, A.shape)

        # Set distance and direction vector to 0
        # print(B[-2][0])
        # TODO: Not exactly correct because last position is here overwritten
        # lastRow = [[A[-2][0], A[-2][1], 0, 0]]
        # offset = np.repeat(lastRow,
        #                    diff, axis=0)
        # A = np.vstack((A, offset))
    A_diff = A[:,2:-1]-B[:,2:-1]
    return A_diff.sum()


def collectDistanceInformation(eval_dir,
                               override=True,
                               metrics=[
                                   "sequenceDirectComparison",
                                   "sequenceContainmentComparison",
                                   "meanWPComparison",
                                   "meanActionVComparison"]):
    '''Collect distance information and write it to diversity.npz.
    If such a file exists and override is not set the data will be loaded from this file.
    Otherwise all data will be collected and stored before returning data and labels
'''

    for m in metrics:
        data = []
        labels = []
        m_path = os.path.join(eval_dir, m + ".npz")
        # Check if file should be overridden or already exists
        if override or not os.path.exists(m_path):
            # search for pattern: _{m}
            for root, dirs, files in os.walk(eval_dir):
                # get specific files and sort them
                # print(files)
                files = [f for f in files if "_" + m + ".csv" in f]
                # n_files = files
                # for fi in n_files:
                #     print("Compare")
                #     print(m)
                #     if search("_" + m, fi):
                #         print("blub", fi)

                # print(files)
                # # sort them
                # exit()
                files.sort(key=lambda f: int(f.split("_")[0]))
                # print(files)
                for f in files:
                    # collect labels and data
                    labels.append(f.split("_")[0])
                    data.append(np.loadtxt(os.path.join(root, f),
                                           delimiter=","))
                break # just in case there are subfolders in result path
            # save all collected data to one file
            data = np.stack(data)
            labels = np.array(labels)
            np.savez(m_path, X=data, y=labels)
            print(m_path)


def loadCollectedInformation(eval_dir, metric):
    '''Load npz file of a specific metric.
    return: data (M,N,N), labels(M,)'''
    m_path = os.path.join(eval_dir, metric + ".npz")
    if not os.path.exists(m_path):
        print("Cannot find", m_path)
        print("Perform a distance calculation!")
        exit()
    runData = np.load(m_path)
    data = runData["X"]
    labels = runData['y']
    return data, labels


def loadAllMetrics(eval_dir,
                   metrics=[
                       "sequenceDirectComparison",
                       "sequenceContainmentComparison",
                       "meanWPComparison",
                       "meanActionVComparison"
                   ]):
    '''Return dictionary of given metrics.
    Crash if one metric file is not existent'''
    coll = {}
    for m in metrics:
        coll[m] = loadCollectedInformation(eval_dir, m)
    return coll


def calculateDistanceInformation(
        pop, idx,
        eval_dir,
        metrics=[
            ("sequenceDirectComparison", sequenceDirectComparison),
            ("sequenceContainmentComparison", sequenceContainmentComparison),
            ("meanWPComparison", meanWPComparison),
            ("meanActionVComparison", meanActionVComparison)
        ], override=False):
    '''Iterate over given metric function and log the resulted distance matrix
    to a csv file in eval_dir.
    The filename will be constructed like follows:
    ==> <idx>_<metric name>.csv'''
    for name, fun in metrics:
        # if idx == 0:
        #     print("Skipping, first round")
        #     break
        filename = str(idx) + "_" + name + ".csv"
        filePath = os.path.join(eval_dir, filename)
        if not override and os.path.exists(filePath):
            print(filename, "already exists!")
            continue
        D = calDistMat(pop, fun)
        print(filename, D.shape)

        np.savetxt(
            filePath,
            D, delimiter=',')
    # print("Done!")

def plot_error(data, label):

    print(data.shape)
    x = np.linspace(0,50, data.shape[0])
    if label % 5 == 0:
        plt.plot(x, data, label=r"$\alpha = {}$".format(label))
    # else:
    #     plt.plot(range(data.shape[0]), data)
    plt.xlim(-1,50)
    plt.ylim(-1,30)
    plt.legend()
    # plt.show()


# https://stackoverflow.com/questions/32791911/fast-calculation-of-pareto-front-in-python
# Faster than is_pareto_efficient_simple, but less readable.
def is_pareto_efficient(costs, return_mask = True):
    """
    Find the pareto-efficient points
    :param costs: An (n_points, n_costs) array
    :param return_mask: True to return a mask
    :return: An array of indices of pareto-efficient points.
        If return_mask is True, this will be an (n_points, ) boolean array
        Otherwise it will be a (n_efficient_points, ) integer array of indices.
    """
    is_efficient = np.arange(costs.shape[0])
    n_points = costs.shape[0]
    next_point_index = 0  # Next index in the is_efficient array to search for
    while next_point_index < len(costs):
        nondominated_point_mask = np.any(costs >= costs[next_point_index], axis=1)
        nondominated_point_mask[next_point_index] = True
        is_efficient = is_efficient[nondominated_point_mask] # Remove dominated points
        costs = costs[nondominated_point_mask]
        next_point_index = np.sum(nondominated_point_mask[:next_point_index])+1
    if return_mask:
        is_efficient_mask = np.zeros(n_points, dtype = bool)
        is_efficient_mask[is_efficient] = True
        return is_efficient_mask
    else:
        return is_efficient


if __name__ == "__main__":
    # import parseRuns as prun
    import matplotlib.pyplot as plt
    import numpy as np
    csv = "/homes/galberding/catkin_ws/devel/lib/ros_optimizer/hello/angletest.csv"
    res = np.genfromtxt(csv, delimiter=",")

    labels = res[:,0]
    res = res[:,1:-1]
    print(res)

    # np.set_printoptions(threshold=np.inf)
    # path = "/homes/galberding/Projects/Ma/evaluation/global_ws/t1/46_train"
    # pools, perfs, run_log = prun.scanRunDir(path)
    # data = []
    # for i, pp in enumerate(pools):
    #     pool = prun.parsePopulationPool(pp)
    #     D = calDistMat(pool, sequenceContainmentComparison)
    #     data.append(D.flatten().mean())
    #     print(i)
    #     if i == 20:
    #         break
    # # plt.boxplot(data)
    # plt.plot(range(len(data)), data)
    # plt.show()

    for i, r in enumerate(res):
        # x = np.linspace(0, 50, res.shape[0])
        print("angle:",labels[i], "m:",r[-1] / 50, "error:", r[-1])
    #     print(r)
    #     print(labels)
    #     plot_error(r, labels[i])
    # plt.show()
