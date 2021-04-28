import commands as com
import json
import operator
import os
import pathlib
import typing

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import tqdm
from matplotlib import pyplot as plt

import eval_gsearch as egss
import parseRuns as prun


cacheDir = pathlib.Path(".cache")


###############################################################################
#                              Gridsearch Methods                             #
###############################################################################


def getConfigString(config):
    return "PM:{}_fun:{}_iAc:{}_iI:{}_sel:{}_keep:{}_tSi:{}_cProb:{}_cLen:{}_cSel:{}_cSt:{}_muta:{}_replG:{}_clearZ:{}_pRot:{}_aP:{}_aPcLenU:{}_aSP:{}_seed:{}".format(
        config["popMin"],
        config["funSelect"],
        config["initActions"],
        config["initIndividuals"],
        config["select"],
        config["keep"],
        config["tournamentSize"],
        config["crossoverProba"],
        config["crossLength"],
        config["crossChildSelector"],
        config["crossStrategy"],
        config["mutaRandScaleDistProba"],
        config["mutaReplaceGen"],
        config["clearZeros"],
        config["penalizeRotation"],
        config["adaptParameter"],
        config["cLenUpper"],
        config["adaptSP"],
        config["genSeed"],
    )


def prepScenario(conf, mapType, maxIter, scenario):
    conf["scenario"] = scenario
    conf["visualize"] = False
    conf["mapType"] = mapType
    # conf["clearZeros"] = 0
    conf["penalizeZeroActions"] = True
    # conf["penalizeRotation"] = True
    conf["maxIterations"] = maxIter


def setConf(
    conf,
    initActions,
    individuals,
    select,
    keep,
    tSize,
    cProba,
    cLen,
    cSel,
    muta,
    muta_gen,
    logDir,
):
    conf["initActions"] = initActions
    conf["initIndividuals"] = individuals
    conf["select"] = select
    conf["keep"] = keep
    conf["tournamentSize"] = tSize
    conf["crossoverProba"] = cProba
    conf["crossLength"] = cLen
    conf["crossChildSelector"] = cSel
    # conf["mutaOrtoAngleProba"] = muta
    conf["mutaRandScaleDistProba"] = muta
    conf["mutaRandAngleProba"] = muta
    conf["mutaReplaceGen"] = muta_gen
    conf["logDir"] = logDir


def getResPaths(conf, scenario=0, mapType=1, gw="gSearch"):
    dirName = conf["logDir"].split("/")[-1]
    gw, sw, res = com.setupEnv(
        "SC:{}_mType:{}".format(conf["scenario"], conf["mapType"]), gw=gw
    )
    res_path = os.path.join(res, dirName)
    return res_path


def resetGW(cons, gw):

    for con in cons:
        name = con["logDir"].split("/")[-1]
        scen = con["logDir"].split("/")[-2]
        # print(name)
        con["logDir"] = os.path.join(gw, scen, name)


def jsonToConf(name, gw=""):
    cDir = cacheDir.joinpath(gw.split("/")[-1]).joinpath(name)
    if cDir.exists():
        with cDir.open() as fin:
            return json.load(fin)
    else:
        return {}

###############################################################################
#                                   Run Generation                             #
###############################################################################


class Run:
    def __init__(self, config, loadPath=False, addRetrain=False):
        """Load Rlog
        ['Iteration 0', 'FitAvg 1', 'FitMax 2', 'FitMin 3',
        'AvgTime 4', 'AvgCoverage 5',
       'ActionLenAvg 6', 'ActionLenMax 7', 'ActionLenMin 8',
        'ZeroActionPercent 9',
       'DeadGens 10', 'BestTime 11', 'BestCov 12',
        'BestLen 13', 'Dmean 14', 'Dstd 15']

        Iteration 0,FitAvg 1,FitMax 2,FitMin 3,
        TimeAvg 4,TimeMax 5,TimeMin 6,
        CovAvg 7,CovMax 8,CovMin 9,
        AcLenAvg 10,AcLenMax 11,AcLenMin 12,
        ZeroAcPercent 13,DGens 14,
        BestTime 15,BestCov 16,BestLen 17,
        DivMean 18,DivStd 19,
        """
        # Load rlog
        self.config = config
        # self.res_path = getResPaths(config, scenario=config["scenario"], mapType=config["mapType"], gw=gw)
        # print("Processing:", config["logDir"])

        # self.data, self.labels = egen.loadAllMetrics(self.res_path, ["sequenceDirectComparison"])["sequenceDirectComparison"]
        performanceFile = "2000_pool.performance.csv"
        actionSnapshotFile = "2000_pool.actions"

        self.bestPath = None
        self.performancePath = os.path.join(config["logDir"], performanceFile)
        self.bestCov = 0
        self.bestTime = 0
        self.popPath = os.path.join(os.path.join(config["logDir"], actionSnapshotFile))
        self.retrain_path = os.path.join(os.path.join(config["logDir"], "retrain_run"))
        if addRetrain:
            self.popPath = os.path.join(self.retrain_path, actionSnapshotFile)


        try:
            with open(os.path.join(config["logDir"], "run_duration"), "r") as f:
                self.duration = float(f.readline())
            self.rlog = pd.read_csv(
                os.path.join(config["logDir"], config["logName"] + ".csv")
            )
            if addRetrain:
                retrain = pd.read_csv(
                    os.path.join(self.retrain_path, config["logName"] + ".csv")
                )
                # update index
                retrain.Iteration = retrain.Iteration.apply(lambda x: x+2000)
                self.rlog = self.rlog.append(retrain)

        except FileNotFoundError:
            print(
                "Cannot find config:",
                os.path.join(config["logDir"], config["logName"] + ".csv"),
            )
            self.rlog = np.zeros((1))
        # self.final_performance = self.rlog[-1, 10] + self.rlog[-1, 11] \
        #     + self.rlog[-1, 4] + self.rlog[-1, 5] # Avg Time/Cov
        # if self.rlog.shape[0] >= config["maxIterations"]:
        if self.rlog.shape[0] >= 2000 and self.rlog.BestObj.iloc[-1] == 0:
            # print(self.rlog.shape)
            self.bestTime = self.rlog.BestTime.iloc[-1]
            self.bestCov = self.rlog.BestCov.iloc[-1]

            self.final_performance = (
                self.rlog.BestCov.iloc[-1] + self.rlog.BestTime.iloc[-1]
            )
            self.fitness = self.rlog.FitMax.iloc[-1]
            if loadPath:
                self.path = self.loadBestPath()
        else:
            self.final_performance = 0

    def loadBestPath(self):
        # Parse population
        # prun.parsePopulation(, name)
        # return prun.parsePopulationPool(self.popPath, perf=self.bestPath).to_array()
        pass

    def pathToTikz(self, idx=""):
            arr = (prun.parsePopulationPool(self.popPath, perf=self.performancePath).to_array())
            actions = np.hstack((arr, np.vstack((arr[1:], arr[0]))))
            tikz = "\\def\\run{}{{".format(idx)
            for row in actions:
                tikz +=  "{}/{}/{}/{},".format(*row)
            tikz = tikz[:-1]
            tikz += "}\n"
            print(tikz)
            return tikz


    def __lt__(self, other):
        return self.final_performance < other.final_performance

    def __eq__(self, o):
        return getConfigString(self.config) == getConfigString(o.config)


def configsToRuns(configs, retrain=False):
    runs = [Run(c) for c in configs]
    runs = []
    for c in configs:
        r = Run(c, addRetrain=retrain)
        if True:
            runs.append(r)

    runs.sort()
    runs = [i for i in runs if i.final_performance > 0]
    return runs




def runsToConfigs(runs: typing.List[Run]):
    return [r.config for r in runs]


def retrainRuns(runs, gw, retrain=2000):
    # pass runs, create the respective directories for it and dunp the configs on the right place as well as setting the right log dirName
    # return configurations that should be retrained or dump them to a json file to restore them for training
    pass

def getBest(runs):
    print(runs)
    cov = time = None
    # runs.sort(key=operator.attrgetter('bestCov'))
    # cov = runs[-1]
    # runs.sort(key=operator.attrgetter('bestTime'))
    # time = runs[-1]
    # bestRuns = [i for i in runs if i.bestCov > 0.7 and i.bestTime > 0.7]
    best = None
    # if bestRuns:
    runs.sort(key=operator.attrgetter('fitness'))
    best = runs[-1]
    return (cov, time, best), runs

###############################################################################
#                           Config selection Methods                       #
###############################################################################


def createFilter(args):
    filters = []
    for k, v in args.items():
        filters.append("{}:{}".format(k, v))
    return filters


def configFilter(c_strs, And=True, **args):
    # print("Passed:", args)
    coll = []
    for st in c_strs:
        ad = And
        if not st:
            print("Empty:", st)
            continue
        for k, v in args.items():
            # print("Check", k, "in", st)
            if And:
                ad &= st[str(k)] == v
            else:
                for val in v:
                    ad |= st[str(k)] == val
        if ad:
            coll.append(st)
            # print("Add Item ---")
    print("{} filter returned {} configs".format("And" if And else "Or", len(coll)))
    return coll


def include_or(**args):
    # print("Pass:", args)

    def inner(func):
        def wrap_filt(confs):
            print("Passing:", args)
            confs = configFilter(confs, False, **args)
            func(confs)

        return wrap_filt

    return inner


def include_and(**args):
    # print("Pass:", args)

    def inner(func):
        def wrap_filt(confs):
            print("Passing:", args)
            confs = configFilter(confs, True, **args)
            func(confs)

        return wrap_filt

    return inner


def getGwCache(gw):
    return cacheDir.joinpath(gw.split("/")[-1])

def nameToPath(gw, name):
    return getGwCache(gw).joinpath(name)


def exploreWorkspace(gw, cache=True):
    """Search for configs in the given workspace.
    If cache is True the configurations are written to a .json file.
    If that file already exists the contents will be loaded and returned instead.
    Set cache to False will override the cache file."""

    cacheFile = "conf_cache.json"
    # cacheFile = pathlib.Path(gw.split("/")[-1]).joinpath(cacheFile)
    cDir = cacheDir.joinpath(cacheFile)
    cDir = nameToPath(gw, cacheFile)

    confs = []
    if not cDir.exists() or not cache:
        for root_, dirs_, files_ in os.walk(gw):
            # print(dirs)
            # print(files)
            # print(root)
            tests = [i for i in dirs_ if "results" not in i]
            for i, d in enumerate(tests):
                print(i, "-->", d)

                for (root, dirs, files) in tqdm.tqdm(os.walk(os.path.join(gw, d))):
                    # print(root)
                    for f in files:
                        if ".yml" in f:
                            cc = com.load_config(os.path.join(root, f))
                            if "genSeed" not in cc:
                                cc["genSeed"] = 42
                            confs.append(cc)
            break
        # Dump configs to file
        updateJson(confs, cacheFile, gw=gw, override=True)
        # with open(cacheFile, "w") as fout:
        #     json.dump(confs, fout)
    else:
        # Return cache content
        # confs = jsonToConf()
        with cDir.open() as fin:
            confs = json.load(fin)

    return confs

def loadPareto(gw):
    cache_file = "pareto_confs.json"
    cDir = getGwCache(gw).joinpath(cache_file)
    if cDir.exists():
        with cDir.open() as fin:
            confs = json.load(fin)
            return confs
    else:
        return {}
    # runs = configsToRuns(confs)
    # egss.scatter_runs(runs)
def loadCacheFile(gw, name):
    # cache_file = "pareto_confs.json"
    cDir = getGwCache(gw).joinpath(name)
    if cDir.exists():
        with cDir.open() as fin:
            confs = json.load(fin)
            return confs
    else:
        print("No cache file {} found".format(name))

def plotPareto(confs, name=""):
    # cc = configFilter(confs)
    runs = configsToRuns(confs)
    egss.scatter_runs(runs, name=name)
    # plt.show()
sns.set()

def scatter_hist(x, y, ax, ax_histx, ax_histy):
    # no labels
    ax_histx.tick_params(axis="x", labelbottom=False)
    ax_histy.tick_params(axis="y", labelleft=False)

    # the scatter plot:
    ax.scatter(x, y)

    # now determine nice limits by hand:
    binwidth = 0.25
    xymax = max(np.max(np.abs(x)), np.max(np.abs(y)))
    lim = (int(xymax/binwidth) + 1) * binwidth

    bins = np.arange(-lim, lim + binwidth, binwidth)
    ax_histx.hist(x, bins=bins)
    ax_histy.hist(y, bins=bins, orientation='horizontal')

def scatter(ax, runs, name=''):
    # plt.rcParams.update({'font.size': 32})
    # plt.figure(figsize=(10, 10))
    # plt.rcParams['axes.facecolor'] = "#F5f5f5"
    # plt.rcParams['axes.axisbelow'] = True
    # all[:, 0]
    all = []
    for run in runs:
        all.append([run.rlog.BestTime.iloc[-1], run.rlog.BestCov.iloc[-1]])

    all = np.array(all)
    # print(all)
    # ax.grid()
    ax.scatter(all[:, 0], all[:, 1])
    # scatter_hist(all[:, 0], all[:, 1], ax, ax_histx, ax_histy)
    # ax.set_xlabel("Time")
    # ax.set_ylabel("Cov")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_title(name, usetex = True)
    # if name:
    #     plt.savefig(name)
    # else:
    # plt.show()

def generalRunPlot(run, name, retrain=False):
    rows = 5
    cols = 1

    sclab = ["Elite", "Turn", "PRWS", "RRWS"]
    fun = run.config["funSelect"]
    mt = run.config["mapType"]
    scen = sclab[run.config["scenario"]]

    fig, axis = plt.subplots(rows, cols, figsize=(10, 10), sharex=True)
    if retrain:
        rlog = run.rlog
    else:
        rlog = run.rlog
    # if cc:
    # print(getConfigString(run.config))
    # axis[0].plot(rlog.Iteration, rlog.FitAvg, label="AvgFit")
    # axis[0].plot(rlog.Iteration, rlog.FitMax, label="MaxFit")
    # axis[0].plot(rlog.Iteration, rlog.FitMin, label="MinFit")
    # axis[0].set_title(r'Population Fitness', usetex = True)
    # axis[0].set_ylim(0,1)
    # axis[0].set_ylabel(r'$f_{}$'.format(fun+1), usetex = True)
    # axis[0].legend()
    row = 0
    axis[row].plot(rlog.Iteration, rlog.BestCov, color='#f60000', label="$cov_{final}$")
    axis[row].plot(rlog.Iteration, rlog.BestTime, color='#00f6f6', label="$t_{final}$")
    # axis[0].plot(rlog.Iteration, rlog.FitMax, label="BestFit")
    axis[row].set_title("Best Performing Individual")
    axis[row].set_ylim(0, 1.1)
    # print((rlog.DivMean-rlog.DivStd).Div)

    row += 1
    axis[row].plot(rlog.Iteration, rlog.TimeAvg, label=r'$avg$')
    axis[row].plot(rlog.Iteration, rlog.TimeMin, label=r'$min$')
    axis[row].plot(rlog.Iteration, rlog.TimeMax, label=r'$max$')
    axis[row].set_ylim(0, 1.1)
    # axis[row].set_ylabel("Diversity", usetex = True)
    axis[row].set_title(r'Population Time, $t_{final}$')

    row += 1
    axis[row].plot(rlog.Iteration, rlog.CovAvg)
    axis[row].plot(rlog.Iteration, rlog.CovMin)
    axis[row].plot(rlog.Iteration, rlog.CovMax)
    axis[row].set_ylim(0, 1.1)
    # axis[row].set_ylabel("Diversity", usetex = True)
    axis[row].set_title(r'Population Coverage, $cov_{final}$')


    row += 1
    axis[row].fill_between(rlog.Iteration.iloc[10:], (rlog.DivMean-rlog.DivStd).iloc[10:], (rlog.DivMean+rlog.DivStd).iloc[10:], alpha=0.3,label="std")
    axis[row].plot(rlog.Iteration.iloc[10:], rlog.DivMean.iloc[10:])
    # axis[row].set_ylabel("Diversity", usetex = True)
    axis[row].set_title(r'Population Diversity, $Div$')

    row += 1
    axis[row].plot(rlog.Iteration, rlog.AcLenAvg)
    axis[row].plot(rlog.Iteration, rlog.AcLenMin)
    axis[row].plot(rlog.Iteration, rlog.AcLenMax)
    axis[row].set_title(r'Population Chromosome Length, $G_{len}$')

    row += 1
    axis[row].plot(rlog.Iteration, rlog.AcLenAvg)
    axis[row].plot(rlog.Iteration, rlog.AcLenMin)
    axis[row].plot(rlog.Iteration, rlog.AcLenMax)
    axis[row].set_title(r'Population Chromosome Length, $G_{len}$')



        # scatter(axis[fun, sc], runs, name=name)
        # egss.runsToDiv(runs, axis_div[fun, sc], name)
    # plt.title(name)
    fig.legend(loc="lower center", fancybox=False, ncol=6)
    plt.setp(axis[-1], xlabel='Iteration')
    # store = os.path.join(savepath, "all_mt:{}_rot_{}.png".format(mt, 1 if penalize else 0))
    # plt.savefig(store)
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.12)
    # plt.show()
    fig.savefig(name)
    # plt.setp(axis[-1, :], xlabel='Time')
    # plt.setp(axis[:, 0], ylabel='Cov')
    # plt.setp(axis_div[-1, :], xlabel='Iteration')
    # plt.setp(axis_div[:, 0], ylabel='Diversity')
    # plt.show()

    # store_scatter = os.path.join(savepath, pref + "mt:{}_rot:{}_sc:{}.png".format(mt, 1 if penalize else 0, sc))
    # store_div = os.path.join(savepath, "div_"+ pref +"mt:{}_rot:{}_sc:{}.png".format(mt, 1 if penalize else 0, sc))
    # fig_div.savefig(store_div)

def plotLenExample(run, name):
    '''Generates Plot with generation duration and chromosome length'''
    rows = 2
    cols = 1

    sclab = ["Elite", "Turn", "PRWS", "RRWS"]
    fun = run.config["funSelect"]
    mt = run.config["mapType"]
    scen = sclab[run.config["scenario"]]

    fig, axis = plt.subplots(rows, cols, figsize=(10, 10), sharex=True)
    rlog = run.rlog
    row = 0
    axis[row].plot(rlog.Iteration, rlog.AcLenAvg)
    axis[row].plot(rlog.Iteration, rlog.AcLenMin)
    axis[row].plot(rlog.Iteration, rlog.AcLenMax)
    axis[row].set_title(r'Population Chromosome Length, $G_{len}$')

    row += 1

    axis[row].plot(rlog.Iteration, rlog.Duration, label=r'$max$')
    axis[row].set_ylim(0, 1000)
    # axis[row].set_ylabel("Diversity", usetex = True)
    axis[row].set_title(r'Iteration Duration [ms]')


    fig.legend(loc="lower center", fancybox=False, ncol=6)
    plt.setp(axis[-1], xlabel='Iteration')
    # store = os.path.join(savepath, "all_mt:{}_rot_{}.png".format(mt, 1 if penalize else 0))
    # plt.savefig(store)
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.12)
    plt.show()
    # fig.savefig(name)

def plotBestSelectionRun(confs, pref="", retrain=False, savepath = "/homes/galberding/Projects/thesis/images/eva/gsearch/best/"):
    ''''''
    rows = 3
    cols = 1
    # sclab = ["Elite", "Turn", "PRWS", "RRWS"]
    for penalize in [False, True]:
        for mt in [1,2]:
            for fun in [0, 1]:
                # fig, axis = plt.subplots(rows, cols, figsize=(12, 4), sharex=True)

                cc = configFilter(confs , mapType=mt, penalizeRotation=penalize, funSelect=fun)
                # confs = configFilter(confs,scenario=scen, funSelect=fun,  mapType=mType)
                runs = configsToRuns(cc)
                # runs = egss.getParetos(runs)

                (cov, time, best), runs_ = getBest(runs)
                name = r'$M_{{type}}={}, f_{{{}}}$, '.format(mt, fun+1)
                generalRunPlot(best, "Test")
 



def plotSelection(confs, pref="", savepath = "/homes/galberding/Projects/thesis/images/eva/gsearch/selection/test"):
    
    rows = 2
    cols = 4
    sclab = ["Elite", "Turn", "PRWS", "RRWS"]
    for penalize in [False, True]:
        for mt in [1,2]:
            fig, axis = plt.subplots(rows, cols, figsize=(12, 4), sharex=True, sharey=True)
            fig_div, axis_div = plt.subplots(rows, cols, figsize=(12, 4), sharex=True, sharey=True)
            for sc in [0,1,2,3]:
                for fun in [0, 1]:
                    cc = configFilter(confs , mapType=mt, scenario=sc, penalizeRotation=penalize, funSelect=fun)
                    # confs = configFilter(confs,scenario=scen, funSelect=fun,  mapType=mType)
                    runs = configsToRuns(cc)
                    name = r'${}, f_{{{}}}$, '.format(sclab[sc], fun+1)
                    scatter(axis[fun, sc], runs, name=name)
                    runs = egss.getParetos(runs)
                    if cc:
                        scatter(axis[fun, sc], runs, name=name)
                        egss.runsToDiv(runs, axis_div[fun, sc], name)
            # plt.show()
            # store = os.path.join(savepath, "all_mt:{}_rot_{}.png".format(mt, 1 if penalize else 0))
            # plt.savefig(store)
            fig_div.tight_layout()
            fig_div.subplots_adjust(bottom=0.2)
            fig_div.legend(labels=["median", "std"],loc="lower center", ncol=2)
            plt.setp(axis[-1, :], xlabel='Time')
            plt.setp(axis[:, 0], ylabel='Cov')
            plt.setp(axis_div[-1, :], xlabel='Iteration')
            plt.setp(axis_div[:, 0], ylabel='Diversity')

            store_scatter = os.path.join(savepath, pref + "mt:{}_rot:{}.png".format(mt, 1 if penalize else 0))
            store_div = os.path.join(savepath, "div_"+ pref +"mt:{}_rot:{}.png".format(mt, 1 if penalize else 0))
            fig.savefig(store_scatter)
            fig_div.savefig(store_div)

def plotSelectionPressure(confs, pref=""):
    savepath = "/homes/galberding/Projects/thesis/images/eva/gsearch/SelectionPressure"
    rows = 2
    cols = 4
    sclab = ["Elite", "Turn", "PRWS", "RRWS"]
    sel_labs = {1:"TS", 3:"SP"}
    for penalize in [False, True]:
        for mt in [1,2]:
            fig, axis = plt.subplots(rows, cols, figsize=(12, 4), sharex=True, sharey=True)
            fig_div, axis_div = plt.subplots(rows, cols, figsize=(12, 4), sharex=True, sharey=True)
            for fun in [0, 1]:
                for col, sc in enumerate([1,3]):
                    for i, sp in (enumerate([1.4, 2],2) if sc == 3 else enumerate([3, 7])):
                        if sc == 3:
                            cc = configFilter(confs , mapType=mt, scenario=sc, penalizeRotation=penalize, funSelect=fun, selPressure=sp)
                        else:
                            cc = configFilter(confs , mapType=mt, scenario=sc, penalizeRotation=penalize, funSelect=fun, tournamentSize=sp)

                        # confs = configFilter(confs,scenario=scen, funSelect=fun,  mapType=mType)
                        runs = configsToRuns(cc)
                        name = r'${}, f_{{{}}}$, ${}={}$'.format(sclab[sc], fun+1, sel_labs[sc], sp)
                        scatter(axis[fun, i], runs, name=name)
                        runs = egss.getParetos(runs)
                        print("I --------------------", i)
                        if cc:
                            scatter(axis[fun, i], runs, name=name)
                            egss.runsToDiv(runs, axis_div[fun, i], name)
            # plt.show()

            fig_div.tight_layout()
            fig_div.subplots_adjust(bottom=0.2)
            fig_div.legend(labels=["median", "std"],loc="lower center", ncol=2)
            plt.setp(axis[-1, :], xlabel='Time')
            plt.setp(axis[:, 0], ylabel='Cov')
            plt.setp(axis_div[-1, :], xlabel='Iteration')
            plt.setp(axis_div[:, 0], ylabel='Diversity')

            store_scatter = os.path.join(savepath, pref + "mt:{}_rot:{}_sc:{}.png".format(mt, 1 if penalize else 0, sc))
            store_div = os.path.join(savepath, "div_"+ pref +"mt:{}_rot:{}_sc:{}.png".format(mt, 1 if penalize else 0, sc))
            fig.savefig(store_scatter)
            fig_div.savefig(store_div)





def plotCrossover(confs, pref=""):
    savepath = "/homes/galberding/Projects/thesis/images/eva/gsearch/crossover/"
    rows = 3
    cols = 4
    for penalize in [False, True]:
        for mt in [1,2]:
            # for sc in [0,1,2,3]:
                # for fun in [0,1]:
                    fig, axis = plt.subplots(rows, cols, figsize=(12, 6), sharex=True, sharey=True)
                    fig_div, axis_div = plt.subplots(rows, cols, figsize=(12.2, 6), sharex=True, sharey=True)
                    for row, clen in enumerate([0.2, 0.4, 0.6, 0.8]):
                        for col, child in enumerate([0, 1, 2]):
                            # cc = configFilter(confs, mapType=mt, crossLength=clen, crossChildSelector=child, penalizeRotation=penalize)
                            cc = configFilter(confs, mapType=mt, crossLength=clen, crossChildSelector=child, penalizeRotation=penalize)
                            runs = configsToRuns(cc)
                            name = r'$C_{{child}}$={}, $C_{{len}}$={}'.format(child, clen)
                            scatter(axis[col, row], runs, name=name)
                            runs = egss.getParetos(runs)
                            print(name)
                            if cc:
                                scatter(axis[col, row], runs, name=name)
                                egss.runsToDiv(runs, axis_div[col, row], name)
                        # plt.title()
                    fig_div.tight_layout()
                    fig_div.subplots_adjust(bottom=0.2)
                    fig_div.legend(labels=["median", "std"],loc="lower center", ncol=2)
                    plt.setp(axis[-1, :], xlabel='Time')
                    plt.setp(axis[:, 0], ylabel='Cov')
                    plt.setp(axis_div[-1, :], xlabel='Iteration')
                    plt.setp(axis_div[:, 0], ylabel='Diversity')
                    store_scatter = os.path.join(savepath, pref+"mt:{}_rot:{}.png".format(mt, 1 if penalize else 0))
                    fig.savefig(store_scatter)
                    store_div = os.path.join(savepath, "div_"+pref+"mt:{}_rot:{}.png".format(mt, 1 if penalize else 0))
                    fig_div.savefig(store_div)



def plot_mutation(confs, pref=""):
    savepath = "/homes/galberding/Projects/thesis/images/eva/gsearch/mutation/"
    rows = 3
    cols = 3
    for penalize in [False, True]:
        for mt in [1,2]:
            # for sc in [0,1,2,3]:
                fig, axis = plt.subplots(rows, cols, figsize=(10, 6), sharex=True, sharey=True)
                fig_div, axis_div = plt.subplots(rows, cols, figsize=(10, 6), sharex=True, sharey=True)
                # fig2, axis2 = plt.subplots(rows, cols, figsize=(10, 6), sharex=True, sharey=True)
                for row,mutA in enumerate([ 0.1, 0.01, 0.0]):
                    for col, mutI in enumerate([0.1, 0.01, 0.0]):
                        cc = configFilter(confs, mapType=mt, penalizeRotation=penalize, mutaRandAngleProba=mutA, mutaRandScaleDistProba=mutA, mutaReplaceGen=mutI)
                        runs = configsToRuns(cc)
                        name = r'$P_{{mA}}$={}, $P_{{mI}}$={}'.format(mutA, mutI)
                        scatter(axis[col, row], runs, name=name)
                        runs = egss.getParetos(runs)
                        print(name)
                        if cc:
                            scatter(axis[col, row], runs, name=name)
                            egss.runsToDiv(runs, axis_div[col, row], name)
                    # plt.title()
                # store = os.path.join(savepath,pref+ "mt:{}_rot_{}_sc:{}.png".format(mt, 1 if penalize else 0), sc)
                # plt.savefig(store)
                # plt.show()

                fig_div.tight_layout()
                fig_div.subplots_adjust(bottom=0.15)
                fig_div.legend(labels=["median", "std"],loc="lower center", ncol=2)
                plt.setp(axis[-1, :], xlabel='Time')
                plt.setp(axis[:, 0], ylabel='Cov')
                plt.setp(axis_div[-1, :], xlabel='Iteration')
                plt.setp(axis_div[:, 0], ylabel='Diversity')

                store_scatter = os.path.join(savepath, pref + "mt:{}_rot:{}.png".format(mt, 1 if penalize else 0))
                store_div = os.path.join(savepath, "div_"+ pref +"mt:{}_rot:{}.png".format(mt, 1 if penalize else 0))
                fig.savefig(store_scatter)
                fig_div.savefig(store_div)

def updateJson(configs, name, override=False, gw=""):
    '''Update or override cached config.'''
    c_confs = []
    u_confs = []
    # cDir = cacheDir.joinpath(gw)
    cDir = getGwCache(gw)
    cDir.mkdir(parents=True, exist_ok=True)
    cDir = cDir.joinpath(name)

    if override:
        with cDir.open("w") as fout:
            json.dump(configs, fout)
    else:
        # check if cachedire exists
        if cDir.exists():
            print("Load and update the configurations")
            with cDir.open() as fin:
                c_confs = json.load(fin)
                # save those not existent in the current config
                for cc in configs:
                    if not any(
                        getConfigString(cc) == getConfigString(c) for c in c_confs
                    ):
                        u_confs.append(cc)
            if len(u_confs) > 0:
                # Write to file:
                with cDir.open("w") as fout:
                    json.dump(c_confs + u_confs, fout)
            else:
                pass
                # print("Noting to update")
        else:
            with cDir.open("w") as fout:
                json.dump(configs, fout)



# from eval_gsearch import scatter_runs

def appendPareto(configs, AND={}, OR={}, gw=""):
    """The inefficient way but it is to be expected
    that all paretos calculated are mutually exclusive
    because they are form different scenarios ..."""

    cache_file = "pareto_confs.json"

    confs = configs.copy()
    if OR:
        confs = configFilter(confs, False, **OR)
    if AND:
        confs = configFilter(confs, True, **AND)

    if not confs:
        print("No Pareto found!")
        return
        # confs = jsonToConf(cacheDir.joinpath(cache_file))
        # return
        # Determain pareto optima
    # else:
    runs = configsToRuns(confs)
    # egss.scatter_runs(runs)

    pa_runs = egss.getParetos(runs)
    # egss.scatter_runs(pa_runs)
    updateJson(runsToConfigs(pa_runs), cache_file, override=False, gw=gw)
    return runs



###############################################################################
#                              Evaluation Methods                             #
###############################################################################
# TODO: Not really a place for that here ...


def moving_average(a, n=5):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1 :] / n


def getBestToTikz(run, tikzFile="out.tex", pref=""):
    confStr = "SC:{}_".format(run.config["scenario"]) + getConfigString(run.config)
    tikzAction = run.pathToTikz(idx=pref)
    with open(tikzFile, "a") as f:
        f.writelines(["% ", confStr, "\n", tikzAction, "\n"])


if __name__ == '__main__':
    # updateJson({"hello": "world"}, "test", override=True)

    # plt_pareto()
    path = "/homes/galberding/catkin_ws/devel/lib/ros_optimizer/ConvexRegion2/2000_pool.actions"
    perf = "/homes/galberding/catkin_ws/devel/lib/ros_optimizer/ConvexRegion2/2000_pool.performance.csv"
    arr = (prun.parsePopulationPool(path, perf=perf).to_array())
    actions = np.hstack((arr, np.vstack((arr[1:], arr[0]))))
    tikz = "\\def\\actions{"
    for row in actions:
        tikz +=  "{}/{}/{}/{},".format(*row)
    tikz += "}"
    print(tikz)
