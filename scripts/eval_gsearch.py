import parseRuns as prun
import eval_gen as egen
import action_vis as avis
import commands as com
from config_temp import config
import os
import numpy as np
from multiprocessing import Pool, cpu_count
import matplotlib.pyplot as plt
import pandas as pd
import subprocess
# from config_manager import getConfigString, moving_average, Run, configsToRuns
import config_manager as cman


###################################################################
#                          Visualization                          #
###################################################################

# def plot_diversity_mean_std(run):

#     data, labels = run.data, run.labels

#     run_median = []
#     run_mean = []
#     run_std = []
#     for i, l in enumerate(labels):
#         data[i][np.triu_indices_from(data[0], 1)]
#         run_median.append(np.median(data[i][np.triu_indices_from(data[i], 1)]))
#         run_mean.append(data[i][np.triu_indices_from(data[i], 1)].mean())
#         run_std.append(data[i][np.triu_indices_from(data[i], 1)].std())
#         # run_std = np.array([data[i][np.triu_indices_from(data[i], 1)].std() for i in range(data.shape[i])])
#         # run_box = np.array([data[i][np.triu_indices_from(data[i], 1)] for i in range(data.shape[i])])
#         # print(run_box.shape)
#         # print(labels.shape)
#         # for idx, b in enumerate(run_box):
#         #     plt.boxplot(b, positions=[int(labels[idx])], showfliers=False)
#     plt.scatter(labels, run_median, s=10, label="median")
#     # plt.plot(labels, run_median, label="median")
#     # plt.plot(labels, run_mean, label=k+"_mean")
#     plt.errorbar(labels, run_mean, run_std, label="mean")
#     # plt.boxplot(run, positions=[count])

###############################################################################
#                              Plotting Functions                             #
###############################################################################
def plotSC1(runs):
    pass


def plot_path_sig_diversity(run):
    """Plot path signature diversity
    """
    rlog = run.rlog
    plt.plot(rlog.Iteration, rlog.DivMean, label="Div")
    # Plot std in background
    plt.fill_between(
        rlog.Iteration,
        rlog.DivMean - rlog.DivStd,
        rlog.DivMean + rlog.DivStd,
        alpha=0.2,
        label="Std",
    )
    plt.legend()


def plot_best_time(run, mean_window=50):
    rlog = run.rlog
    plt.plot(
        cman.moving_average(rlog.Iteration.array, n=mean_window),
        cman.moving_average(rlog.BestTime.array, n=mean_window),
        label="BestTime",
    )


def plot_best_cov(run, mean_window=50):
    rlog = run.rlog
    plt.plot(
        cman.moving_average(rlog.Iteration.array, n=mean_window),
        cman.moving_average(rlog.BestCov.array, n=mean_window),
        label="BestCov",
    )


def plot_time(run, mean_window=50):
    rlog = run.rlog
    plt.plot(
        cman.moving_average(rlog.Iteration.array, n=mean_window),
        cman.moving_average(rlog.TimeAvg.array, n=mean_window),
        label="TimeAvg",
    )


def plot_cov(run, mean_window=50):
    rlog = run.rlog
    plt.plot(
        cman.moving_average(rlog.Iteration.array, n=mean_window),
        cman.moving_average(rlog.CovAvg.array, n=mean_window),
        label="AvgCov",
    )


def plot_gen_length(run):
    rlog = run.rlog
    plt.plot(rlog.Iteration, rlog.AcLenAvg, label="AvgLen")
    plt.plot(rlog.Iteration, rlog.AcLenMax, label="MaxLen")
    plt.plot(rlog.Iteration, rlog.AcLenMin, label="MinLen")
    # TODO: Best len
    plt.legend()


def plot_dead_gens(run):
    rlog = run.rlog
    print(rlog.shape)
    plt.plot(rlog.Iteration, rlog.DGens)
    plt.title("Dead Gens")
    plt.xlabel("Iteration")
    plt.ylabel("Dead Gens")



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

def scatter_runs(runs, name=""):
    plt.rcParams.update({'font.size': 32})
    plt.figure(figsize=(10, 10))
    plt.rcParams['axes.facecolor'] = "#F5f5f5"
    plt.rcParams['axes.axisbelow'] = True
    all = []
    for run in runs:
        all.append([run.rlog.BestTime.iloc[-1], run.rlog.BestCov.iloc[-1]])

    res = all = np.array(all)
    print(all)
    plt.grid(True, linewidth=3)
    plt.scatter(all[:, 0], all[:, 1])
    plt.xlabel("Time")
    plt.ylabel("Cov")
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    if name:
        plt.savefig(name)
    else:
        plt.show()

def scatter(ax, runs):
    plt.rcParams.update({'font.size': 32})
    plt.figure(figsize=(10, 10))
    plt.rcParams['axes.facecolor'] = "#F5f5f5"
    plt.rcParams['axes.axisbelow'] = True
    all = []
    for run in runs:
        all.append([run.rlog.BestTime.iloc[-1], run.rlog.BestCov.iloc[-1]])

    res = all = np.array(all)
    print(all)
    ax.set_grid(True, linewidth=3)
    ax.scatter(all[:, 0], all[:, 1])
    # ax.set_xlabel("Time")
    # ax.set_ylabel("Cov")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    # if name:
    #     plt.savefig(name)
    # else:
    # plt.show()



def getParetos(runs):
    all = []

    for run in runs:
        all.append([run.rlog.BestTime.iloc[-1], run.rlog.BestCov.iloc[-1]])
        # plt.scatter(run.rlog.TimeAvg[-1:], run.rlog.CovAvg[-1:])

    # print(all)
    all = np.array(all)
    # print(egen.is_pareto_efficient(all))
    # exit()
    return [runs[idx] for idx, val in enumerate(egen.is_pareto_efficient(all)) if val]



def scatter_all(runs):
    all = []
    for run in runs:
        all.append([run.rlog.BestTime.iloc[-1], run.rlog.BestCov.iloc[-1]])
        # plt.scatter(run.rlog.TimeAvg[-1:], run.rlog.CovAvg[-1:])

    print(all)
    res = all = np.array(all)
    print("Test:", all.shape)
    sel = egen.is_pareto_efficient(all)
    all = all[sel]
    print(all.shape)
    plt.scatter(res[:, 0], res[:, 1])
    plt.scatter(all[:, 0], all[:, 1])

    plt.xlabel("Time")
    plt.ylabel("Cov")
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.grid(True)


def plot_fitness(run):
    rlog = run.rlog
    plt.plot(rlog.Iteration, rlog.FitAvg, label="AvgFit")
    plt.plot(rlog.Iteration, rlog.FitMax, label="MaxFit")
    plt.plot(rlog.Iteration, rlog.FitMin, label="MinFit")
    plt.legend()


def plotRun(run):
    print("Visualizing:", cman.getConfigString(run.config))
    # if run.rlog.shape[0] == 1:
    #     print("Not enough datapoints for printing")
    #     return
    # plt.title(getConfigString(config))
    # print(run.rlog[0])
    rows = 6
    cols = 1
    count = 1
    # Diversity plot (sequence comparison)
    plt.subplot(rows, cols, count)
    # plot_diversity_mean_std(run)
    win = 1
    plot_best_cov(run, win)
    plot_best_time(run, win)
    plt.legend()
    # plt.ylim(0, 1)

    # Plot Path signature diversity
    count += 1
    plt.subplot(rows, cols, count)
    plot_path_sig_diversity(run)

    # Time, Cov plot
    count += 1
    window = 1
    plt.subplot(rows, cols, count)
    plot_cov(run, window)
    plot_time(run, window)
    plt.legend()
    plt.ylim(0, 1)

    # Gen fitness plot
    count += 1
    plt.subplot(rows, cols, count)
    plot_fitness(run)

    # Gen length plot
    count += 1
    plt.subplot(rows, cols, count)
    plot_gen_length(run)

    # Dead gen plot
    count += 1
    plt.subplot(rows, cols, count)
    plot_dead_gens(run)
    plt.ylim(0, config["initIndividuals"])

    fig = plt.gcf()
    fig.set_size_inches((30, 30))
    fig.canvas.set_window_title(cman.getConfigString(run.config))
    plt.show()


def visualize(runs, bestN=10, gw="gSearch"):
    # scatter_all(runs)
    plt.show()

    for run in runs:
        plotRun(run)
        plt.figure(1)
        # plt.figure(2)
        avis.plot_best_run(run.config["logDir"])
        plt.title("#" + str(bestN))
        # bestN -= 1
        plt.show()
        # break


# def ev_gsearch(gw):

#     # configs = readConfigs(gw="/homes/galberding/Projects/Ma/gridsearch")
#     configs = readConfigs(gw=gw)
#     # configs = [configs[i] for i in range(10)]
#     runs = configsToRuns(configs)
#     visualize(runs, bestN=3)


def loadAndEvalConfigs(configs):
    runs = cman.configsToRuns(configs)
    # visualize(runs, bestN=3)
    plot_diversity(runs)
    plt.show()


def newM(M, d, d1, d4):
    gamma = M * d
    if d1 < d4:
        return M + gamma
    if d1 > d4:
        return M - gamma
    else:
        return M


def vis_pop_change():
    M = 30
    M_down = 10
    M_up = 150
    D = np.linspace(0.5, 0.1, 100)


def rank(p, n=100, SP=1.8):
    return 2 - SP + 2 * (SP - 1) * ((p - 1) / (n - 1))


def plot_diversity(runs):
    all = []
    print("All Runs:", len(runs))
    for run in runs:
        all.append([run.rlog.BestTime.iloc[-1], run.rlog.BestCov.iloc[-1]])
        plt.plot(run.rlog.Iteration, run.rlog.DivMean)


# def eva(gw):
#     # gw = "/homes/galberding/Projects/Ma/gridsearch/adaptBaselineCross1"
#     ev_gsearch(gw)


def fuse_configs_to_df(confs, colNames, col="DivMean"):
    runs = cman.configsToRuns(confs)
    df = pd.DataFrame()
    cc = 0

    for r in runs:
        cName = " ".join([str(r.config[i]) for i in colNames])
        if r.rlog.BestCross.iloc[-1] > 200:
            continue
        df.insert(loc=cc, column=cc, value=r.rlog[col])
        cc += 1
    print(df)
    # df = df.rolling(100).mean()
    df = df.iloc[1:]

    df_mean = df.mean(axis=1)
    df_std = df.std(axis=1)
    df_med = df.median(axis=1)
    print(type(df_std))
    ddf = pd.DataFrame({"mean":df_mean, "std":df_std, "median": df_med})
    # ddf.insert(loc=0, column="mean", value=df_mean)

    # df_mean.plot()
    # df_std.plot()
    ax = ddf.reset_index().plot(x='index', y = 'mean', yerr="std")
    ddf.reset_index().plot(x="index", y="median", ax=ax)
    plt.show()

def runsToDiv(runs, ax, name):
    df = pd.DataFrame()
    cc = 0

    for r in runs:
        # cName = " ".join([str(r.config[i]) for i in colNames])
        # if r.rlog.BestCross.iloc[-1] > 200:
        #     continue
        df.insert(loc=cc, column=cc, value=r.rlog["DivMean"])
        cc += 1
    print(df)
    # df = df.rolling(100).mean()
    df = df.iloc[4:]

    df_mean = df.mean(axis=1)
    df_std = df.std(axis=1)
    df_med = df.median(axis=1)
    print(type(df_std))
    ddf = pd.DataFrame({"mean":df_mean, "std":df_std, "median": df_med})
    # ddf.insert(loc=0, column="mean", value=df_mean)

    # df_mean.plot()
    # df_std.plot()
    # ddf.reset_index().plot(x='index', y ='mean', yerr="std", ax=ax)
    print(df_mean.to_numpy().shape)
    # ax.errorbar(range(len(df_mean)), df_mean, yerr=df_std)
    ax.fill_between(range(len(df_mean)), df_mean-df_std, df_mean+df_std, alpha=0.3,label="std")
    # ax.plot(range(len(df_mean)), df_mean)
    ax.plot(range(len(df_mean)), df_med, label="median")
    # ax.set_xlabel("Time")
    # ax.set_ylabel("Cov")
    ax.set_title(name, usetex = True)
    ax.label_outer()
    # ddf.reset_index().plot(x="index", y="median", ax=ax)
    # plt.show()



if __name__ == "__main__":
    gw = "/homes/galberding/Projects/Ma/gridsearch/adaptBaselineCross1"
    # ev_gsearch(gw)
    # d = 0.3
    # d1 = 0.3
    # d4 = 0.4
    # count = 0
    # for i in range(0,101):
    #     count += rank(i)
    #     print(rank(i))
    #     print(count)
