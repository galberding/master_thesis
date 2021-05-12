import commands as com
import os
import pathlib
import subprocess
import time
from multiprocessing import Pool, cpu_count
from shutil import copy

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import tqdm

import action_vis as avis
# from config_manager import *
import config_manager as cman
import eval_gen as egen
import eval_gsearch as egs
import parseRuns as prun
from config_temp import config

counter = 0

rlog_name = "rlog.npz"
div_name = "diversity.npz"

###############################################################################
#                                    Helper                                   #
###############################################################################


###############################################################################
#                               Conf Generation                               #
###############################################################################

def prepRetrainConfigs(runs, gw, exe="", prep=True, store=True, retrain=2000, cacheName="retrain.json"):
    '''Always resets the config to the given workspace.
    Set retrain to defined value.
    Create directories in the given global worspace if enabled
If enabled store them in cache'''
    # Reset workspace
    # Copy configs to new directory
    # start training

    confs = cman.runsToConfigs(runs)
    # reset runs

    cman.resetGW(confs, gw)
    for conf in confs:

        conf["retrain"] = retrain
        conf["exe"] = os.path.join(gw, "opti")
    # Create all needed directories
    if prep:
        for conf in confs:
            name = "SC:{}_mType:{}".format(conf["scenario"], conf["mapType"])
            gw, sw, res = com.setupEnv(name, gw=gw)


    if exe:
        copy(exe, os.path.join(gw, "opti"))


            # com.setupEnv(name, gw)

        # Copy exe and set path in config

    if store:
        cman.updateJson(confs, cacheName, override=True, gw=gw)


def trainScenario(scenario, mapType, maxIter, gw, dynamic=False, dynSel=False, rotPanel=False, exe=""):
    global config
    conf = config

    # Important adaptations

    conf["penalizeRotation"] = rotPanel
    conf["penalizeZeroActions"] = False
    conf["clearZeros"] = 1
    # conf["crossStrategy"] = 1
    conf["fitSselect"] = 1
    conf["popMin"] = 100
    conf["mapResolution"] = 0.2
    conf["adaptSP"] = dynSel if scenario > 0 else False

    fun_ = [0, 1] # 1
    c_strat_ = [0]

    # Scenario 0
    initActions_ = [50]
    individuals_ = [100]
    tSize_ = [0]
    select_ = [0]
    keep_ = [0]
    cProba_ = [0.8]
    cLen_ = [0.2,0.4,0.6,0.8]
    # cLen_ = [0.2, 0.3]
    cSel_ = [0, 1, 2] #2
    # muta_ = [0.0]
    muta_ = [0.0, 0.01, 0.1]
    # muta_gen_ = [0.0]
    muta_gen_ = [0.0, 0.01, 0.1]

    if scenario > 0:
        individuals_ = [100]
        select_ = [10, 20]
        keep_ = [1, 10]

    if scenario == 1:  # Tournament
        tSize_ = [3, 7]
        if dynSel:
            tSize_ = [2]
            conf["adaptSPlower"] = 1
            conf["adaptSPupper"] = 10

    elif scenario == 3:  # RRWS
        tSize_ = [1.4, 2]
        if dynSel:
            conf["adaptSPlower"] = 1.1
            conf["adaptSPupper"] = 2

    if dynamic:
        conf["adaptParameter"] = True
        conf["crossUpper"] = 1
        conf["crossLower"] = 0.6
        conf["cLenUpper"] = 0.6
        conf["cLenLower"] = 0.6

        cProba_ = [0.8]
        cLen_ = [0.6]
        muta_gen_ = [0]
        # muta_gen_ = [0.1, 0.01, 0.001]
        muta_ = [0]

    configs = []

    name = "SC:{}_mType:{}".format(scenario, mapType)
    gw, sw, res = com.setupEnv(name, gw=gw)

    if exe:
        conf["exe"] = os.path.join(gw, "opti")
        copy(exe, conf["exe"])

    cman.prepScenario(conf, mapType, maxIter, scenario)
    # print(conf["logDir"])
    count = 0

    for initActions in initActions_:
        for individuals in individuals_:
            for tSize in tSize_:
                for select in select_:
                    for keep in keep_:
                        for cProba in cProba_:
                            for cLen in cLen_:
                                for cSel in cSel_:
                                    for muta in muta_:
                                        for muta_gen in muta_gen_:
                                            for fun in fun_:
                                                for c_strat in c_strat_:
                                                    conf["mutUpper"] = muta_gen

                                                    if scenario == 0:
                                                        conf["popMin"] = individuals
                                                    if scenario > 0:
                                                        conf["popMin"] = individuals
                                                        # individuals  = 1000
                                                    # else:
                                                    #     conf["popMin"] = 2 * select
                                                    conf["cLenLower"] = cLen
                                                    conf["funSelect"] = fun
                                                    conf["crossStrategy"] = c_strat

                                                    # setConf(conf, initActions, individuals, select, keep, tSize, cProba, cLen, cSel, muta, muta_gen, os.path.join(sw, getConfigString(conf)))
                                                    if scenario > 0:
                                                        cman.setConf(
                                                            conf,
                                                            initActions,
                                                            1000,
                                                            select,
                                                            keep,
                                                            tSize,
                                                            cProba,
                                                            cLen,
                                                            cSel,
                                                            muta,
                                                            muta_gen,
                                                            os.path.join(
                                                                sw,
                                                                cman.getConfigString(
                                                                    conf
                                                                ),
                                                            ),
                                                        )
                                                    else:
                                                        cman.setConf(
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
                                                            os.path.join(
                                                                sw,
                                                                cman.getConfigString(
                                                                    conf
                                                                ),
                                                            ),
                                                        )
                                                    if scenario == 3:
                                                        conf["selPressure"] = tSize
                                                        conf["tournamentSize"] = tSize
                                                    else:
                                                        conf["selPressure"] = 0
                                                    configs.append(conf.copy())
    return configs


# M#

# MM#


###############################################################################
#                                   Training                                  #
###############################################################################


def execute(conf):
    timeout = 2400  # 40min

    conf_path = os.path.join(conf["logDir"], "conf.yml")
    # print(conf_path)
    # Create config path
    if not os.path.exists(conf["logDir"]):
        os.makedirs(conf["logDir"])
    com.save_config(conf, conf_path)
    start = time.time()
    try:
        subprocess.run([conf["exe"], conf_path], timeout=timeout)
    except subprocess.TimeoutExpired:
        print("Timeout:", conf["logDir"])
    duration = time.time() - start
    with open(os.path.join(conf["logDir"], "run_duration"), "w") as f:
        f.write(str(duration))
    # global counter
    # counter += 1
    # print(conf["logDir"].split("/")[-1], "==>", counter, "DONE")
    return conf, duration


# def train(configs):
#     with Pool(cpu_count()) as p:
#         print(p.map(execute, configs))


###########################################################################
#                                Evaluation                               #
###########################################################################


@cman.include_or(scenario=[1])
@cman.include_and(
    crossStrategy=0,
    # scenario=1,
    # crossoverProba=0.9,
    # crossLength=0.,
    # funSelect=3
    # crossChildSelector=2,
    # mutaReplaceGen=0,
    # mutaRandAngleProba=0,
    mutaRandScaleDistProba=0
)
def exploreDiv(con):
    # loadAndEvalConfigs(con)
    # print("Configs:", con["logDir"])
    egs.fuse_configs_to_df(
        con,
        [
            "scenario",
            "crossLength",
            "crossStrategy",
            "crossoverProba",
            "mutaRandScaleDistProba",
            "mutaRandAngleProba",
            "mutaReplaceGen",
        ],
        col="DivMean",
    )

def executeConfigs(confs):
    with Pool(CPU_CORES) as p:
        for conf, duration in tqdm.tqdm(
                p.imap_unordered(execute, confs), total=len(confs)
        ):
            # print(getConfigString(conf), "--", duration)
            pass


def train(scenario, gw, exe, test=True, rotPanel=False):
    # for scenario in [3]:
    for mType in [1, 2]:
        confs = trainScenario(
            scenario, mType, 2000, gw, dynamic=False, dynSel=False, rotPanel=rotPanel, exe=exe
        )
        print("Train SC: {}, MType: {}, Overall: {}"
              .format(scenario, mType, len(confs)))
        if not test:
            with Pool(CPU_CORES) as p:
                for conf, duration in tqdm.tqdm(
                        p.imap_unordered(execute, confs), total=len(confs)
                ):
                    # print(getConfigString(conf), "--", duration)
                    pass


def savePareto(gw):
    con = cman.exploreWorkspace(gw)
    for mType in [1,2]:
        for scenario in [0,1,2,3]:
            for fun in [0,1]:
                for child in [0,1,2]:
                    for clen in [0.1, 0.2,0.3,0.4,0.5, 0.6]:
                        for mutA in [0.0, 0.001, 0.01, 0.1]:
                            for mutI in [0.0, 0.001, 0.01, 0.1]:
                                cman.appendPareto(
                                    con,
                                    {
                                        # And:
                                        # "crossStrategy": 1,
                                        "scenario": scenario,
                                        # "penalizeRotation": True,
                                        "mapType": mType,
                                        "funSelect": fun,
                                        "crossChildSelector": child,
                                        "crossStrategy": 0,
                                        # "crossoverProba": 0.9,
                                        "crossLength": clen,
                                        # "funSelect": 3,
                                        # "crossChildSelector": 2,
                                        # "mutaReplaceGen": 0,
                                        "mutaRandAngleProba": mutA,
                                        "mutaRandScaleDistProba": mutA,
                                        "mutaReplaceGen": mutI
                                    },
                                    {
                                        # Or
                                        # "scenario": scenario
                                    },
                                    gw=gw
                                )
         # runs= cman.configsToRuns(cman.jsonToConf(paretoDir))
         # egs.scatter_runs(runs)
         # plt.show()


def runPathToLog(run, savepath, idx):
    descr = "SC:{}_".format(run.config["scenario"])+cman.getConfigString(run.config)
    fname = "path_mt_{}.tex".format(run.config["mapType"])

    with open(os.path.join(savepath, fname), "a") as f:
        f.writelines([
            "% " + descr + "\n",
            run.pathToTikz(idx=chr(ord("A") + idx)),
            "\n"
        ])

def createDirs(*args):
    for path in args:
        if not os.path.exists(path):
            os.makedirs(path)

if __name__ == "__main__":
    import socket
    global_workspace = "/homes/galberding/Gridsearch"
    exe = "/homes/galberding/catkin_ws/devel/lib/ros_optimizer/opti"

    gw = os.path.join(global_workspace, "currentWorkspace")
    gw_retrain = os.path.join(global_workspace, "retrain")
    result_dir = os.path.join(global_workspace, "res")
    result_best_path_dir = os.path.join(result_dir, "bestPath")
    result_best_path_retrain_dir = os.path.join(result_dir, "bestPathRetrain")
    selection_dir = os.path.join(result_dir, "selection")
    selectionPressure_dir = os.path.join(result_dir, "selectionPressure")
    crossover_dir = os.path.join(result_dir, "crossover")
    mutation_dir = os.path.join(result_dir, "mutation")

    imgSaveDir = "/homes/galberding/Projects/thesis/images/eva/funcs/"
    retrainGw = ""

    createDirs(gw, imgSaveDir, gw_retrain, result_dir, selectionPressure_dir, selection_dir, crossover_dir, mutation_dir, result_best_path_dir, result_best_path_retrain_dir)

    import sys
    from eval_gsearch import loadAndEvalConfigs, fuse_configs_to_df
    import eval_gsearch as egs
    paretoDir = pathlib.Path(".cache/pareto_confs.json")
    CPU_CORES = cpu_count() * 2
    # CPU_CORES = 50
    print(sys.argv)
    if len(sys.argv) > 1:
        # if sys.argv[-1] == "e":
        # ev_gsearch(gw)
        if sys.argv[-1] == "a":
            con = cman.exploreWorkspace(gw)
            if socket.gethostname() == "schorschi":
                cman.resetGW(con, gw)
            exploreDiv(con)
        elif sys.argv[1] == "i":  # index files to cache
            cman.exploreWorkspace(gw, cache=False)
        elif sys.argv[1] == "s":
            # if len(sys.argv) > 2:
                # for scen in [s for i, s in enumerate(sys.argv) if i > 2]:
            savePareto(gw)
        elif sys.argv[1] == "p":
            confs_all = cman.exploreWorkspace(gw)
            confs_par = cman.loadPareto(gw)
            if len(sys.argv) == 3:
                if "selection"== sys.argv[2]: # without rotationmapType
                    cc = cman.configFilter(confs_all)
                    cman.plotSelection(cc, pref="all_", savepath=selection_dir)
                elif "crossover"== sys.argv[2]: # without rotationmapType
                    cc = cman.configFilter(confs_all)
                    cman.plotCrossover(cc, pref="all_", savepath=crossover_dir)
                elif "mutation"== sys.argv[2]: # without rotationmapType
                    cc = cman.configFilter(confs_all)
                    cman.plot_mutation(cc, pref="all_", savepath=mutation_dir)
                elif "selPressure"== sys.argv[2]: # without rotationmapType
                    cc = cman.configFilter(confs_all)
                    cman.plotSelectionPressure(cc, pref="all_", savepath=selectionPressure_dir)
                # elif "bestRun"== sys.argv[2]: # without rotationmapType
                #     cc = cman.configFilter(confs_par)
                #     cman.plotBestSelectionRun(cc, pref="all_")
                else:
                    print("Nothing to do")

        elif sys.argv[1] == "retrain":
            confs_all = cman.exploreWorkspace(gw)
            confs_par = cman.loadPareto(gw)
            # gw_retrain = "/homes/galberding/retrainGridsearch"
            cacheFile = "retrain.json"
            if sys.argv[-1] == "prep":
                bestRuns = []
                for penalize in [False, True]:
                    for mt in [1,2]:
                        for fun in [0, 1]:
                            for sc in [0,1,2,3]:
                                cc = cman.configFilter(confs_all , mapType=mt, penalizeRotation=penalize, funSelect=fun, scenario=sc)

                                # confs = configFilter(confs,scenario=scen, funSelect=fun,  mapType=mType)
                                runs = cman.configsToRuns(cc)
                                (cov, time, best), runs_ = cman.getBest(runs)
                                bestRuns.append(best)
                # bestRuns = cman.configsToRuns(cman.loadCacheFile(gw_retrain, cacheFile))
                # cman.resetGW()
                prepRetrainConfigs(bestRuns, gw_retrain, exe=exe, cacheName=cacheFile)
            elif sys.argv[-1] == "run":
                # load cache file
                confs_ret = cman.loadCacheFile(gw_retrain, cacheFile)
                print(len(confs_ret))
                executeConfigs(confs_ret)
                # runs = egss.getParetos(runs)
            elif sys.argv[-1] == "plotBest":
                confs_ret = cman.loadCacheFile(gw_retrain, cacheFile)
                runs = cman.configsToRuns(confs_ret)
                cman.generalRunPlot(runs[0], "first")

            elif len(sys.argv) == 3:
                confs_ret = cman.loadCacheFile(gw_retrain, cacheFile)
                pref = "ret_"
                if "selection"== sys.argv[2]: # without rotationmapType
                    cc = cman.configFilter(confs_ret)
                    cman.plotSelection(cc, pref=pref, savepath=selection_dir, retrain=True)
                elif "crossover"== sys.argv[2]: # without rotationmapType
                    cc = cman.configFilter(confs_ret)
                    cman.plotCrossover(cc, pref=pref, savepath=crossover_dir)
                elif "mutation"== sys.argv[2]: # without rotationmapType
                    cc = cman.configFilter(confs_ret)
                    cman.plot_mutation(cc, pref=pref, savepath=mutation_dir)
                elif "selPressure"== sys.argv[2]: # without rotationmapType
                    cc = cman.configFilter(confs_ret)
                    cman.plotSelectionPressure(cc, pref=pref, savepath=selectionPressure_dir)
                # elif "bestRun"== sys.argv[2]: # without rotationmapType
                #     cc = cman.configFilter(confs_par)
                #     cman.plotBestSelectionRun(cc, pref="all_")
                else:
                    print("Nothing to do")
        elif sys.argv[1] == "duration":
            # Print table of durations for each run in seconds
            confs_all = cman.exploreWorkspace(gw)
            # confs_par = cman.loadPareto(gw)
            time_ = {}
            size = {}
            tests = "\#Runs "
            for sc in [0,1,2,3]:
                time_[sc] = []
                # for mt in [1,2]:

                cc = cman.configFilter(confs_all ,  scenario=sc)
                tests += " & " + str(len(cc))
            # print(tests)
                # runs = cman.configsToRuns(cc)
                for con in cc:
                    run = cman.configsToRuns([con])
                    if len(run) < 1:
                        continue
                    run = run[0]
                    time_[sc].append(run.duration)

            tableavg = "DurationAvg "
            tableMin = "MinDuration "
            tableMax = "MaxDuration "
            for i in range(4):

                tt = np.array(time_[i])
                # print(tt)
                tableavg += " & " + str(tt.mean())
                tableMin += " & " + str(tt.min())
                tableMax += " & " +str(tt.max())

            print(tableavg)
            print(tableMax)
            print(tableMin)

        elif sys.argv[1] == "bestTable":
            newGw = gw_retrain
            cacheFile = "retrain.json"
            confs_ret = cman.loadCacheFile(newGw, cacheFile)
            # savePath = "/homes/galberding/Projects/thesis/images/eva/gsearch/best/"
            savePath = result_best_path_dir
            print(len(confs_ret))
            # Reset basic configuration
            cman.resetGW(confs_ret, gw)
            # iterate over each run
            table = {}

            plotDir = os.path.join(savePath,"runPlots")
            if not os.path.exists(plotDir):
                os.makedirs(plotDir)


            labs = ["Elite", "Turn", "PRWS", "RRWS"]
            for mt in [1,2]:
                table[mt] = {}
                runiter = 0
                confs = cman.configFilter(confs_ret, mapType=mt)
                for con in confs:
                    print("Co")
                    run = cman.configsToRuns([con])
                    if not run:
                        continue
                    run = run[0]
                    runPathToLog(run, savePath, runiter)
                    table[mt][runiter] = "({}) ".format(chr(ord("a") + runiter))
                    # Add attributes to table
                    table[mt][runiter] += " & " + str(6.28 if run.config["penalizeRotation"] == 1 else 0)
                    table[mt][runiter] += " & $f_{{{}}}$".format(run.config["funSelect"]+1)
                    table[mt][runiter] += " & " + str(labs[run.config["scenario"]])
                    # table[mt][runiter] += " & " + str(run.config["mapType"])
                    table[mt][runiter] += " & " + str(run.config["keep"])
                    table[mt][runiter] += " & " + str(run.config["select"])
                    table[mt][runiter] += " & " + str(run.config["tournamentSize"])
                    table[mt][runiter] += " & " + str(run.config["selPressure"])
                    table[mt][runiter] += " & " + str(run.config["crossLength"])
                    table[mt][runiter] += " & " + str(run.config["crossChildSelector"])
                    table[mt][runiter] += " & " + str(run.config["mutaRandAngleProba"])
                    table[mt][runiter] += " & " + str(run.config["mutaReplaceGen"])
                    table[mt][runiter] += " & " + str(run.bestCov)
                    table[mt][runiter] += " & " + str(run.bestTime)
                    cman.generalRunPlot(run, os.path.join(plotDir,"mt:{}_{}.png".format(mt, chr(ord("a") + runiter))))
                    runiter += 1
                    # buildTable

            print("GEnerate table 1 --------------------")
            for k,v in table[1].items():
                print(v)
            print("GEnerate table 2 --------------------")
            for k,v in table[2].items():
                print(v)

        elif sys.argv[1] == "bestTableRetrain":
            newGw = gw_retrain
            cacheFile = "retrain.json"
            confs_ret = cman.loadCacheFile(newGw, cacheFile)
            savePathRetrainContent = result_best_path_retrain_dir

            print(len(confs_ret))
            # Reset basic configuration
            # cman.resetGW(confs_ret, gw)
            # iterate over each run
            table = {}
            plotDir = os.path.join(savePathRetrainContent,"runPlots")
            if not os.path.exists(plotDir):
                os.makedirs(plotDir)

            labs = ["Elite", "Turn", "PRWS", "RRWS"]
            for mt in [1,2]:
                table[mt] = {}
                runiter = 0
                confs = cman.configFilter(confs_ret, mapType=mt)
                for con in confs:
                    print("Co")
                    run = cman.configsToRuns([con], retrain=True)
                    if not run:
                        continue
                    run = run[0]
                    runPathToLog(run, savePathRetrainContent, runiter)
                    table[mt][runiter] = "({}) ".format(chr(ord("a") + runiter))
                    # Add attributes to table
                    table[mt][runiter] += " & " + str(6.28 if run.config["penalizeRotation"] == 1 else 0)
                    table[mt][runiter] += " & $f_{{{}}}$".format(run.config["funSelect"]+1)
                    table[mt][runiter] += " & " + str(labs[run.config["scenario"]])
                    # table[mt][runiter] += " & " + str(run.config["mapType"])
                    table[mt][runiter] += " & " + str(run.config["keep"])
                    table[mt][runiter] += " & " + str(run.config["select"])
                    table[mt][runiter] += " & " + str(run.config["tournamentSize"])
                    table[mt][runiter] += " & " + str(run.config["selPressure"])
                    table[mt][runiter] += " & " + str(run.config["crossLength"])
                    table[mt][runiter] += " & " + str(run.config["crossChildSelector"])
                    table[mt][runiter] += " & " + str(run.config["mutaRandAngleProba"])
                    table[mt][runiter] += " & " + str(run.config["mutaReplaceGen"])
                    table[mt][runiter] += " & " + str(run.bestCov)
                    table[mt][runiter] += " & " + str(run.bestTime)



                    cman.generalRunPlot(run, os.path.join(plotDir,"mt:{}_{}.png".format(mt, chr(ord("a") + runiter))))
                    runiter += 1
                    # buildTable

            print("GEnerate table 1 --------------------")
            for k,v in table[1].items():
                print(v)
            print("GEnerate table 2 --------------------")
            for k,v in table[2].items():
                print(v)

        elif sys.argv[1] == "t":
            if len(sys.argv) == 3:
                train(int(sys.argv[2]), gw, exe, test=False, rotPanel=False)
                train(int(sys.argv[2]), gw, exe, test=False, rotPanel=True)
                # train(int(sys.argv[2]), gw, exe, test=False, rotPanel=True)

            else:
                print("No scenario provided!")
        else:
            print("No Valid option")
