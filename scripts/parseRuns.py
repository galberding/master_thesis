import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import math
conf = {
    "maxIterations": 0,
    "initIndividuals": 0,
    "selectIndividuals": 0,
    "selectKeepBest": 0,
    "time": 0,
    "occ": 0,
    "coverage": 0
}


def dirToAngle(pos):
    return math.atan2(pos[1], pos[0]) * (180 / math.pi)


def strToConfig(logStr, logVals, config):
    # print(logStr)
    # print(logVals)
    # vals =
    for k, v in zip(logStr.split(','), logVals.split(',')):
        if len(k) == 0:
            continue
        # print(k, v)
        config[k] = float(v)


def fillRunLog(idxList, csvLog, config):
    for row in csvLog:
        # print()
        for i, v in enumerate(row[:-1].split(",")):
            config[idxList[i]].append(float(v))


def settingsToStr(settings):
    strl = []
    for k, v in settings.items():
        if k == "maxIterations" or k == "selectKeepBest" or k == "initIndividuaE501 line too long (83 > 79 characters)ls":
            continue
        strl.append(k + ":" + str(v))
    return "_".join(strl)


def depricated():
    # Read results of given path:
    # res_path = "results/GAV2_all_mutations"
    res_path = "results/v2_mutation_roulett/"
    # res_path = "results/v2_mutation_roulett_timeWeightedByCoverage/"
    # res_path = "results/v2_mutation_selectBest_timeWeightedByCoverage/"
    for root, dirs, files in os.walk(res_path):
        for f in files:
            with open(os.path.join(root,f), 'r') as log:
                # Read lines into the dicts
                # get config which is located at the second line:
                lines = log.readlines()
                if len(lines[2]) <= 1:
                    continue
                conf = {
                    "maxIterations": 0,
                    "initIndividuals": 0,
                    "selectIndividuals": 0,
                    "selectKeepBest": 0,
                    "time": 0,
                    "occ": 0,
                    "coverage": 0
                }
                idxList = lines[2][:-1].split(",")
                strToConfig(lines[0][:-1], lines[1][:-1], conf)
                runLog = {"settings": conf,
                          "Iteration": [],
                          "FitAvg": [],
                          "FitMax": [],
                          "FitMin": [],
                          "AvgTime": [],
                          "AvgOcc": [],
                          "AvgCoverage": [],
                          "ActionLenAvg": [],
                          "ActionLenMax": [],
                          "ActionLenMin": []
                          }

                # Insert values to runLog
                fillRunLog(idxList, lines[3:], runLog)
                # if (sum(runLog["AvgCoverage"]) / len(runLog["AvgCoverage"])) < 0.1:
                #     continue
                plt.plot(runLog[idxList[0]], runLog[idxList[1]], label="Fitness")
                plt.plot(runLog[idxList[0]], runLog["AvgTime"], label="Time")
                plt.plot(runLog[idxList[0]], runLog["AvgOcc"], label="Occ")
                plt.plot(runLog[idxList[0]], runLog["AvgCoverage"], label="Coverage")
                # plt.plot(runLog[idxList[0]], runLog["AvgCoverage"], label="Coverage")
                # print(runLog["                plt.plot(runLog[idxList[0]], runLog["AvgCoverage"], label="Coverage")"])
                plt.title(settingsToStr(runLog["settings"]))
                plt.legend()
                print(runLog["settings"])
                plt.show()


class Action():
    def __init__(self, actionStr):
        astr = actionStr.split("|")
        self.type = int(astr[0])
        a = astr[1].split(":")
        if(len(astr) > 2):
            b = astr[2].split(":")
            self.b = [float(b[0]), float(b[1])]
        else:
            # set b to the same value as a
            self.b = [float(a[0]), float(a[1])]
        self.a = [float(a[0]), float(a[1])]


class Path():
    '''Parser to convert a string to path
    When generating the first WP of each action is used
    TODO: Evaluate if there is a difference between first and last WP
    '''
    def __init__(self, actionStr):
        actions = actionStr.split(",")
        self.al = []
        self.info = None
        for action in actions:
            self.al.append(Action(action))

    def to_array(self):
        arr = []
        for a in self.al:
            arr.append(a.a)
        return np.array(arr)

    def getMeans(self):
        arr = self.to_array()
        return arr.mean(axis=0)

    def actionToArr(self, action: Action):
        p1 = np.array(action.a)
        p2 = np.array(action.b)
        V = p2 - p1
        dist = np.linalg.norm(V)
        angle = 0
        if dist:
            angle = dirToAngle(V / dist)
        # print("P1:", p1, "P2:", p2, "Angle:", angle, "Dist:", dist)
        return np.array([p1[0], p1[1], p2[0], p2[1]])

    def describeActions(self, maxLen):
        actionInfo = []
        for ac in self.al:
            actionInfo.append(self.actionToArr(ac))
        for i in range(maxLen - len(self.al)):
            actionInfo.append(np.array([0,0]))
        return np.stack(actionInfo)

    def actionWaypoints(self):
        actionInfo = []
        for ac in self.al:
            actionInfo.append(self.actionToArr(ac))



    def actionInfos(self):
        if self.info is not None:
            return self.info
        self.info = []
        for ac in self.al:
            # start point and V
            self.info.append(self.actionToArr(ac))
        self.info = np.stack(self.info)
        return self.info

    def compare(self, p):
        '''Compare two paths with each other'''
        A = p.actionInfos()
        B = self.actionInfos()
        if len(self.al) > len(p.al):
            A = self.actionInfos()
            B = p.actionInfos()


        minSum = 0
        for b in B:
            A_diff = A - b
            A_norms = np.linalg.norm(A_diff[:, 0:2], axis=1) \
                + np.linalg.norm(A_diff[:, 2:-1], axis=1)
            # print(A_norms)
            minSum += np.min(A_norms)
            # exit()
        # print(minSum)
        return minSum


def getBest(path):
    bestIdx = 0
    bestFit = 0
    log = np.loadtxt(path, comments='#',delimiter=',',skiprows=1)
    idx = np.argmax(log, axis=0)
    print(log[idx[0]])
    return idx[0]


def parsePopulation(path, name):
    return parsePopulationPool(os.path.join(path, name))


def parsePopulationPool(path, perf=""):
    '''Load *pool.actions file, parse the content
    and return the described population '''
    population = []
    with open(path, "r") as f:
        lines = f.readlines()
        for line in lines:

            p = Path(line)
            # Split to actions
            population.append(p)
    if perf:
        idx = getBest(perf)
        print("REturn best")
        return population[idx]
    print("Return Pop")
    return population


def getSimilarActions(path: Path):
    '''Return the appearance of each action'''
    # TODO: Counting is not correct,
    #       Each action is counted again even if it is counted as already appearing
    arr = path.to_array()
    check = {}
    for i in range(arr.shape[0]):
        checkval = np.isclose(arr,arr[i])
        # log = sum(checkval)

        check[i] = (sum(np.logical_and(checkval[:,0], checkval[:,1])), arr[i])
    print(check)
    for k, v in check.items():
        # summ all duplicate up
        print(k, v[0])


def getRunIdx(runname: str):
    return int(runname.split("_")[0])


def scanRunDir(path: str, include_iter=10):
    '''get action and performance data,
    return sorted path in run order as tuple:
(list of pools, list of performance data)'''
    for root, dirs, files in os.walk(path):
        perfs = []
        pools = []
        run_log = None
        for f in files:
            # print(f)
            if "actions" in f:
                if getRunIdx(f) % include_iter == 0:
                    pools.append(os.path.join(path, f))
            elif "run" in f:
                run_log = os.path.join(path, f)
            elif "perform" in f:
                if getRunIdx(f) % include_iter == 0:
                    perfs.append(os.path.join(path, f))

        return pools, perfs, run_log


def parsePerfLogFile(path: str):
    return np.loadtxt(path,comments='#',delimiter=',',skiprows=1)


def parseRunLogFile(sw: str):
    # TODO: Update parameter list
    # Iteration,FitAvg,FitMax,FitMin,AvgTime,AvgCoverage,ActionLenAvg,ActionLenMax,ActionLenMin,Zeros,BestTime,BestCov
    name = "run.log.csv"
    return np.loadtxt(os.path.join(sw, name),comments='#',delimiter=',',skiprows=1)


def loadCSVToNumpy(path: str):
    print("Load:", path)
    return np.loadtxt(path ,comments='#',delimiter=',',skiprows=1)


def analizeRun(path: str):
    '''Iterate over run dir and calculate std and mean values for the actions as well as further infos'''
    pools, perfs, run_log = scanRunDir(path)
    print(run_log)
    runPools = []
    runPoolMeans = []
    runPoolStd = []
    for p_path in pools:
        pool = []
        # print(p_path)
        for gen in parsePopulationPool(p_path):
            pool.append(gen.getMeans())
        # print(pool)
        runPoolMeans.append(np.stack(pool))
        # runPools.append(parsePopulationPool(p_path))
    return runPoolMeans


def saveRunList(runMeans):
    np.savetxt("test.out", runMeans, delimiter=",")


def getMaxPoolActionLen(pool: [Path]):
    '''Return the max action sequence length.'''
    maxLen = 0
    for p in pool:
        length = len(p.al)
        if length > maxLen:
            maxLen = length
    return maxLen


def calActionMinDist(pool):
    l = len(pool)
    distMat = np.zeros((l,l))
    for row in range(l):
        for col in range(l):
            if distMat[row, col] == 0:
                distMat[row, col] = \
                    distMat[col, row] = pool[row].compare(pool[col])
    return distMat




if __name__ == "__main__":
    path = "/homes/galberding/catkin_ws/devel/lib/ros_optimizer/elitistSel/"
    # pool = parsePopulation(path, "100000_pool.actions")
    # for pa in pool:
    #     getSimilarActions(pa)
    #     break
    # getBest(path, "100000_pool.performance.csv")
    # pools = analizeRun(path)
    # print(pools)
    # for i, p in enumerate(pools):
    #     plt.scatter(p[:,0], p[:,1], label=str(i))
    # plt.legend()
    # plt.show()

    pools, perfs, run_log = scanRunDir(path)
    for perf in perfs:
        print(getRunIdx(perf.split("/")[-1]))
        pf = parsePerfLogFile(perf)
        print(pf)

    plt.subplot(2, 1, 1)
    ll = parseRunLogFile(run_log)
    # plt.plot(ll[:,0], ll[:,1], label="avg fitness")
    # plt.plot(ll[:,0], ll[:,4], label="avg time")
    # plt.plot(ll[:,0], ll[:,6], label="avg cov")
    plt.stackplot(ll[:,0] )
    plt.xlabel("Iteration")
    plt.ylabel("Fitness")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(ll[:,0], ll[:,-4], label="avg action len")
    plt.plot(ll[:,0], ll[:,-2], label="min action len")
    plt.plot(ll[:,0], ll[:,-3], label="max action len")
    plt.legend()
    plt.xlabel("Iteration")
    plt.ylabel("#Actions")
    plt.show()




    # saveRunList(pools)
