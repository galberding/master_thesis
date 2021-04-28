'''Workspace Manager for executing and configuration of executables'''

# import os
import argparse
from commands import create_sws, \
    create_from, \
    list_sws, \
    conf_list_parameters, \
    conf_edit_parameters, \
    train, \
    evaluate_list_run_infos, \
    evaluator
import commands as com

def main():
    # Main parser
    parser = argparse.ArgumentParser()
    parser.add_argument("--gw", help="Path to global workspace",
                        type=str, default="global_ws")
    parser.add_argument("--config", "-c",
                        help="Path to config.yml",
                        type=str, default="config.yml")
    parser.add_argument("sw", help="Subworkspace name", type=str)
    parser.add_argument("-e", "--executable",
                        type=str, default="/homes/galberding/catkin_ws/devel/lib/ros_optimizer/opti")

    parser.set_defaults(func=list_sws)
    # Subparser
    sub_parser = parser.add_subparsers(
        title="Manage Subworkspaces",
        help="pass name for subworkspace")

    # Workspace creation
    # > wsm <sw> create|c
    create_parser = sub_parser.add_parser(
        "create",
        aliases=['c'],
        help="create help")
    # create_parser.add_argument("sw", type=str,
    #                            help="Create Subworkspace")
    create_parser.set_defaults(func=create_sws)
    # Clone existing sw
    # > wsm create|c <sw> from <sw_old>
    csub = create_parser.add_subparsers()
    clone_parser = csub.add_parser("from", help="Create sw from existing sw")
    clone_parser.add_argument("sw_old", help="Existing Workspace", type=str)
    clone_parser.set_defaults(func=create_from)

    # Listing sws
    # > wsm list|l
    list_parser = sub_parser.add_parser("list", aliases=['l'],
                                        help="list all subworkspaces")
    list_parser.set_defaults(func=list_sws)

    # List parameter in configuration file
    # > wsm conf list
    conf_parser = sub_parser.add_parser(
        "conf",
        aliases=['c'],
        help="create help")
    conf_sub_parser = conf_parser.add_subparsers()
    conf_list_parser = conf_sub_parser.add_parser(
        "list",
        aliases=["l"],
        help="List configurable parameter")
    conf_list_parser.set_defaults(func=conf_list_parameters)

    # Edit entry
    # wsm <sw> conf e <id> <value>
    conf_edit_parser = conf_sub_parser.add_parser(
        "edit",
        aliases=["e"],
        help="Edit entry")
    conf_edit_parser.add_argument("entry_id", type=int, help="Id of config entry")
    conf_edit_parser.add_argument("conf_value", type=str, help="New config value")
    conf_edit_parser.set_defaults(func=conf_edit_parameters)

    # Training
    # wsm <sw> train
    train_parser = sub_parser.add_parser(
        "train",
        aliases=['t'],
        help="Optimize path in sw")
    train_parser.set_defaults(func=train)

    # Evaluation
    # prepare and evaluate all aspects for plotting
    eval_parser = sub_parser.add_parser(
        "eval",
        aliases=['ev'],
        help="Evaluate generated runs")
    eval_sub_parser = eval_parser.add_subparsers()
    # > wsm <sw> eval list
    eval_list_parser = eval_sub_parser.add_parser(
        "list",
        aliases=['l'],
        help="List information about the current runs")
    eval_list_parser.set_defaults(func=evaluate_list_run_infos)

    # > wsm <sw> eval run <iter>
    eval_run_parser = eval_sub_parser.add_parser(
        "run", help="List information about the current runs")
    eval_run_parser.add_argument(
        "run_id",
        type=int,
        help="Run ID, use list for possible ids")
    eval_run_parser.add_argument("--all", "-a", action='store_true', help="Calculate all metrics")
    eval_run_parser.add_argument("--contain", "-c", action='store_true', help="Compare actions based on minimal dist to other ")
    eval_run_parser.set_defaults(func=com.evalCalDistances)
    # eval_run_parser.set_defaults(func=evaluator)
    # eval_run_subparser = eval_run_parser.add_subparsers()
    # eval_calculate_distance_parser = eval_run_subparser.add_parser(
    #     "dist",
    #     aliases=["d"],
    #     help="Calculate all distance metrics"
    # )
    # eval_calculate_distance_parser.set_defaults(func=com.evalCalDistances)

    # Plotting
    # > wsm <sw> plot list
    plot = {}
    plot["plot"] = sub_parser.add_parser(
        "plot",
        aliases=['pl'],
        help="Plot generated runs")
    plot_sub_parser = plot["plot"].add_subparsers()
    plot["list"] = plot_sub_parser.add_parser(
        "list",
        aliases=['l'],
        help="List information about the current runs")
    plot["list"].set_defaults(func=evaluate_list_run_infos)

    # > wsm <sw> plot run <x>
    plot["run"] = plot_sub_parser.add_parser(
        "run", help="List information about the current runs")
    plot["run"].add_argument(
        "run_id",
        type=int,
        help="Run ID, use list for possible ids")
    plot["run"].add_argument("--all", "-a",
                             action='store_true', help="Calculate all metrics")
    # plot["run"].add_argument("--contain", "-c",
    #                          action='store_true',
    #                          help="Compare actions based on minimal dist to other ")

    plot["run"].set_defaults(func=com.plottingCommand)


    # plot_diversity_parser = plot_sub_parser.add_parser(
    #     "div", help="Plot diversity")
    # plot_diversity_parser.add_argument(
    #     "run_id",
    #     type=int,
    #     help="Run ID, use list for possible ids")
    # plot_diversity_parser.set_defaults(func=com.plotDiversity)
    # Parsing
    args = parser.parse_args()
    args.func(args)


main()
