'''Lib that provides commands that can be called by wsm'''
import os
from shutil import copyfile
import subprocess
import matplotlib.pyplot as plt
import numpy as np
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

import parseRuns as prun
import visRuns as vrun
import eval_gen as evalg
###############################################################################
#                           General Helper Functions                          #
###############################################################################


def check_exit(path, msg):
    '''Exit and print message if path does not exist.'''
    if not os.path.exists(path):
        print(msg)
        exit()


def createPath(path):
    if not os.path.exists(path):
        os.makedirs(path)


def getConfigName(path):
    '''Return the name of the configuration file in a sw'''
    for root, dirs, files in os.walk(path):
        for f in files:
            if "yml" in f and "~" not in f:
                return f
    print("Cannot find configuration file in: " + path)
    exit()


def load_config(path):
    '''Load yaml config and return dictionary'''
    with open(path, "r") as f:
        data = load(f, Loader=Loader)
        # print(data)
        return data


def save_config(conf, path):
    '''Write config back to file'''
    # print(conf)
    with open(path, "w") as f:
        dump(conf, f, Dumper=Dumper)


def paths(args):
    '''Prebuild absolute paths to gw, sw and config.'''
    current = os.getcwd()
    gw = os.path.join(current, args.gw)
    sw = os.path.join(gw, args.sw)
    config = os.path.join(sw, args.config)
    return gw, sw, config


def genDirPaths(args):
    gw, sw, conf_path = paths(args)
    run_name = str(args.run_id)+"_train"
    run_path = os.path.join(sw, run_name)
    res_path = os.path.join(sw, "results", run_name)
    return gw, sw, run_path, res_path, conf_path


def setupEnv(name, gw="/homes/galberding/Projects/Ma/gridsearch", res="results"):
    current = os.getcwd()
    gw = os.path.join(current, gw)
    res = os.path.join(gw, res, name)
    sw = os.path.join(gw, name)
    if not os.path.exists(gw):
        os.makedirs(gw)
    if not os.path.exists(sw):
        os.makedirs(sw)
    if not os.path.exists(res):
        os.makedirs(res)
    # print(sw)
    return gw, sw, res


def get_dir_count(path):
    '''Count train directories'''
    count = 0
    for root, dirs, files in os.walk(path):
        for d in dirs:
            if "_train" in d:
                count += 1
        return count


###############################################################################
#                                   Commands                                  #
###############################################################################


def check_create_global(path):
    '''Create global workspace'''
    if not os.path.exists(path):
        print("Create new global workspace")
        os.makedirs(path)


def list_sws(args):
    check_create_global(args.gw)
    # print("Found workspaces")
    for root, dirs, files in os.walk(args.gw, ):
        print("Subworkspaces:")
        for d in dirs:
            print("*", d)
        print("Total:", len(dirs))
        break


def create_sws(args, config_path=None):
    print("Create Workspace:", args.sw, "in", args.gw)
    # print(args)
    if not config_path:
        config_path = args.config
    check_create_global(args.gw)
    sw_dir = os.path.join(args.gw, args.sw)
    # Create sw
    try:
        os.mkdir(sw_dir)
    except Exception:
        print("Workspace already exists!\nOverriding config!")
        # exit()
    # Insert configuration file
    check_exit(config_path, "Cannot find Configuration file: " + config_path)
    print(config_path)
    copyfile(config_path, os.path.join(sw_dir, config_path.split("/")[-1]))


def create_from(args):
    '''Copy configuration file from old sw and create a new one.'''
    check_exit(os.path.join(args.gw, args.sw_old),
               "No sw with name " + args.sw_old + " found")
    sw_old = os.path.join(args.gw, args.sw_old)
    conf_name = getConfigName(sw_old)
    create_sws(args, config_path=os.path.join(sw_old, conf_name))


def conf_list_parameters(args):
    '''List all parameters that are found in the sw configuration.'''
    gw, sw, conf_path = paths(args)
    config = load_config(conf_path)
    for i, (k, v) in enumerate(config.items()):
        print(i, "-->", k, ":", v)


def conf_edit_parameters(args):
    '''Edit one value in the configuration file.'''
    gw, sw, conf_path = paths(args)
    config = load_config(conf_path)
    key = None
    for i, (k, v) in enumerate(config.items()):
        if args.entry_id == i:
            key = k
            break
    if key:
        print("Change:", key, ":", config[key], "to", args.conf_value)
        config[key] = args.conf_value
    else:
        # print("Cannot find entry")
        return
    save_config(config, conf_path)
    print("Done")


def train(args):
    gw, sw, conf_path = paths(args)
    # get name of ssw
    ssw = os.path.join(sw, str(get_dir_count(sw)) + "_train")
    os.makedirs(ssw)
    ssw_conf = os.path.join(ssw, conf_path.split("/")[-1])
    # Copy config to ssw
    copyfile(conf_path, ssw_conf)
    # Edit config to ensure logging is correct
    conf = load_config(ssw_conf)
    conf["logDir"] = ssw
    conf["restore"] = False
    save_config(conf, ssw_conf)
    # execute training:
    print("Start Training")
    subprocess.run([args.executable, ssw_conf])
    # subprocess.run(["echo", ssw_conf])


def evaluate_list_run_infos(args):
    gw, sw, conf_path = paths(args)
    different_runs = get_dir_count(sw)
    print("Found ", different_runs, "runs:")
    for i in range(different_runs):
        ssw = os.path.join(sw, str(i) + "_train")
        pools, perfs, run_log = prun.scanRunDir(ssw, include_iter=1)
        print("\t", i, "-->", len(perfs), "snapshots")


def saveSequenceDist(pools):
    '''Handle Action by action comparison.'''
    for pp in pools:
        # print(pp)
        idx = prun.getRunIdx(pp.split("/")[-1])
        print(idx)
        pop = prun.parsePopulationPool(pp)
        vrun.visDistanceMatrix(pop)
        # vrun.visDistanceMatrix(pop, save=os.path.join(res_path,str(idx)))


def comparePathsIndividualActionDist(pools, res_path):
    for pp in pools:
        # print(pp)
        idx = prun.getRunIdx(pp.split("/")[-1])
        print(idx)
        pop = prun.parsePopulationPool(pp)
        vrun.visDistanceMatrixIndividualAction(
            pop,
            save=res_path,
            idx=str(idx)
        )




def evaluator(args):

    # run_id, save,
    gw, sw, run_path, res_path, conf_path = genDirPaths(args)
    createPath(res_path)
    # print(run_path)
    check_exit(run_path, "Cannot find run directory!")
    pools, perfs, run_log = prun.scanRunDir(run_path)
    # print(pools)
    comparePathsIndividualActionDist(pools, res_path)


def evalCalDistances(args):
    # all, contain
    gw, sw, run_path, res_path, conf_path = genDirPaths(args)
    createPath(res_path)
    check_exit(run_path, "Cannot find run directory!")
    metrics = []
    if args.all:
        # add all metrics to the list
        metrics = []
    pools, perfs, run_log = prun.scanRunDir(run_path)
    for pp in pools:
        idx = prun.getRunIdx(pp.split("/")[-1])
        pool = prun.parsePopulationPool(pp)
        evalg.calculateDistanceInformation(pool, idx, res_path)
    evalg.collectDistanceInformation(res_path)


def plotDiversity(args):

    # np.set_printoptions(threshold=np.inf)
    gw, sw, run_path, res_path, conf_path = genDirPaths(args)
    # Check if diversity mat was created, if not calculate it
    runRes = evalg.loadAllMetrics(res_path)
    rlog = prun.parseRunLogFile(run_path)

    count = 1
    labs = []
    for k, (data, labels) in runRes.items():
        labs = labels
        # for i in range(10,len(labels)):
        # if labels[i] % 1000:
        # idx.append(i)
        # real_idx.append(labels[i])
        # print(data.shape)
        # print(labels.shape)
        plt.subplot(5,1,count)
        print(labels)
        run_median = np.array([np.median(data[i][np.triu_indices_from(data[0], 1)]) for i in range(data.shape[0])])
        run_mean = np.array([data[i][np.triu_indices_from(data[0], 1)].mean() for i in range(data.shape[0])])
        run_std = np.array([data[i][np.triu_indices_from(data[0], 1)].std() for i in range(data.shape[0])])
        run_box = np.array([data[i][np.triu_indices_from(data[0], 1)] for i in range(data.shape[0])])
        print(run_box.shape)
        print(labels.shape)
        # for idx, b in enumerate(run_box):
        #     plt.boxplot(b, positions=[int(labels[idx])], showfliers=False)
        plt.scatter(labels, run_median, s=10, label="median")
        # plt.plot(labels, run_median, label="median")
        # plt.plot(labels, run_mean, label=k+"_mean")
        plt.errorbar(labels, run_mean, run_std, label="mean")
        # plt.boxplot(run, positions=[count])
        plt.title(k)
        count += 1


        plt.legend()


    print()
    labs = labs.astype(int)
    fitness_avg = rlog[labs][:,1]
    fitness_max = rlog[labs][:,2]
    fitness_min = rlog[labs][:,3]
    plt.subplot(5,1,5)
    plt.plot(labels, fitness_max, label="max")
    plt.plot(labels, fitness_min, label="min")
    plt.plot(labels, fitness_avg, label="avg")
    plt.title("Fitness")
    plt.legend()
    plt.subplots_adjust(hspace=0.6)
    plt.show()
    # plt.plot(real_idx, rlog[real_idx][:, 2])

    # plt.xticks(rotation=45)

    # run = data[0]
    # print("Std:",run.std())
    # print("Median:",np.median(run))
    # print("Mean:",run.mean())
    # print("Quantile 75:",np.quantile(run, 0.75))
    # print("Quantile 25:",np.quantile(run, 0.25))

    # TODO: Plot mean against fitness


# def moving_average(x, w):
#     return np.convolve(x, np.ones(w), 'valid') / w



def plottingCommand(args):
    '''Plot fitness and stuff of current run'''
    gw, sw, run_path, res_path, conf_path = genDirPaths(args)
    rlog = prun.parseRunLogFile(run_path)
    n=1001
    # plt.plot(rlog[:,0], rlog[:,4], label="Time")
    # plt.plot(rlog[:,0], rlog[:,4], label="Time")
    plt.plot(rlog[:-n+1,0], moving_average(rlog[:,4], n=n), label="Time")
    plt.plot(rlog[:-n+1,0], moving_average(rlog[:,6], n=n), label="Cov")
    plt.legend()
    plt.title(sw)
    plt.show()
