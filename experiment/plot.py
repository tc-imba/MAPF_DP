import json

from matplotlib import rcParams
import matplotlib
matplotlib.use('Agg')

# rcParams['font.family'] = 'serif'
# rcParams['font.sans-serif'] = ['Times']

import ast
import dataclasses
from pathlib import Path
from typing import List, Any
import click
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


from matplotlib.ticker import ScalarFormatter, LogLocator, LogFormatter
import numpy as np
from motor.motor_asyncio import AsyncIOMotorClient
from tqdm import tqdm
import scipy.stats as stats
import scipy.integrate
from loguru import logger

from experiment.app import app_command, AppArguments
from experiment.utils import asyncio_wrapper, project_root, ExperimentSetup, validate_list

# rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
# rc('font',**{'family':'serif','serif':['Times']})
# rc('text', useTex=False)

# LINE_COLORS = ["#5BB8D7", "#57A86B", "#A8A857", "#0a2129", "#FF0000", "#330066"]
jet = plt.get_cmap('jet')
LINE_COLORS_NUMBER = 8
color_norm = matplotlib.colors.Normalize(vmin=0, vmax=LINE_COLORS_NUMBER)
scalar_map = matplotlib.cm.ScalarMappable(norm=color_norm, cmap=jet)
LINE_COLORS = [scalar_map.to_rgba(i) for i in range(LINE_COLORS_NUMBER)]

LINE_MARKERS = ["o", "s", "^", "+", "x", "*", ".", ",", "v", "1"]

# plt.rcParams['font.family'] = 'serif'
# plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['text.usetex'] = True

mongo_client = AsyncIOMotorClient()
db = mongo_client["MAPF_DP"]
parsed_collection = db["parsed"]


def format_log(x, pos=None):
    x = float(x)
    if x.is_integer():
        return str(int(x))
    return str(x)


@dataclasses.dataclass
class PlotArguments(AppArguments):
    map_names: List[str]
    plot_dir: Path


@dataclasses.dataclass
class LineSettings:
    simulator: str
    label: str
    linestyle: str
    color: str = ""
    marker: str = ""


@dataclasses.dataclass
class PlotSettings:
    args: PlotArguments
    plot_type: str
    subplot_type: str
    agents: int
    plot_value: str
    y_field: str
    groupby: List[str]

    extra: str = ""
    legend: bool = True

    # automatically initialized
    subplot_field: str = ""
    subplot_keys: List[Any] = dataclasses.field(default_factory=list)
    y_log: bool = False
    y_label: str = ""
    x_field: str = ""
    x_label: str = ""

    def __post_init__(self):
        # self.y_log = False
        if self.y_field == "time":
            if self.plot_type == "simulator" or self.plot_type == "replan":
                if self.args.timing == "continuous":
                    self.y_label = 'Average Computation Time (ms) \n per Time Unit Simulated'
                else:
                    self.y_label = 'Average Computation Time \n of Each Timestep (ms)'
            elif self.plot_type == "feasibility":
                self.y_label = 'Average Computation Time of Each Feasibility Check (ms)'
            elif self.plot_type == "cycle":
                self.y_label = 'Average Computation Time \n of Each Timestep (ms)'
                # self.y_log = True
            else:
                assert False
        elif self.y_field == "makespan":
            self.y_label = '\n Makespan'
        elif self.y_field == "average_cost":
            self.y_label = '\n Sum of Costs'
        elif self.y_field == "makespan_time":
            self.y_label = 'Avg. Comp. Time \n of Each Timestep (ms)'
            if self.plot_type == "cycle":
                self.y_log = True
        elif self.y_field == "plan_percent":
            self.y_label = "Plan Percentage (\\%)"
        elif self.y_field == "cdf":
            self.y_label = "\n Timesteps Completed (\\%)"
        elif self.y_field == "pdf":
            self.y_label = "\n Timesteps Completed (\\%)"
        elif self.y_field == "node_pair":
            self.y_label = "\n Number of Node Pairs"
            self.y_log = True
        else:
            assert False

        if self.plot_type == "cdf":
            self.x_label = "Computation Time (s)"
            self.x_field = ""
        elif self.plot_type == "redundant":
            self.x_label = "$2^k$ connected"
            self.x_field = "k_neighbor"
        else:
            self.x_label = 'length of each pause (k)'
            self.x_field = 'delay_interval'

        if self.subplot_type == "delay-ratio":
            self.subplot_field = "delay_ratio"
            self.subplot_keys = self.args.delay_ratios
        elif self.subplot_type == "obstacles":
            self.subplot_field = "obstacles"
            self.subplot_keys = self.args.obstacles
        elif self.subplot_type == "map-names":
            self.subplot_field = "map_name"
            self.subplot_keys = self.args.map_names
        elif self.subplot_type == "delay-interval":
            self.subplot_field = "delay_interval"
            self.subplot_keys = self.args.delay_intervals
        elif self.subplot_type == "k_neighbor":
            self.subplot_field = "k_neighbor"
            self.subplot_keys = self.args.k_neighbors
        else:
            assert False

    def get_subplot_key(self, subplot_key):
        if self.subplot_type == "delay-ratio" or self.subplot_type == "delay-interval" or self.subplot_type == "k_neighbor":
            subplot_key = int(subplot_key)
        elif self.subplot_type == "obstacles":
            subplot_key = f"{int(subplot_key * 100)}\\%"
        elif self.subplot_type == "map-names":
            subplot_key = str(subplot_key)
        else:
            assert False
        return subplot_key

    def get_line_settings(self, indexes):
        if self.plot_type == "simulator":
            (simulator, cycle, subplot_key) = indexes
            subplot_key = self.get_subplot_key(subplot_key)
            if cycle != "proposed":
                cycle = "naive"
            if simulator == "default":
                label = f"baseline-{subplot_key}"
            elif simulator.startswith("replan"):
                # label = f"{simulator}-{subplot_key}"
                label = f"replan-{subplot_key}"
            elif simulator == "pibt":
                label = f"causal-pibt+-{subplot_key}"
            elif simulator == "prioritized":
                label = f"prioritized-{subplot_key}"
            elif simulator == "snapshot":
                label = f"snapshot-none-{subplot_key}"
            elif simulator == "snapshot_start":
                label = f"snapshot-start-{subplot_key}"
            elif simulator == "snapshot_end":
                label = f"offline-{subplot_key}"
            elif simulator.startswith("online_remove_redundant"):
                if self.y_field == "time":
                    label = f"{'-'.join(simulator.split('_'))}-{subplot_key}"
                else:
                    label = f"online-{subplot_key}"
            elif simulator.startswith("online"):
                arr = simulator.split("_")
                arr[0] = "proposed"
                prefix = "-".join(arr)
                label = f"{prefix}-{subplot_key}"
            elif simulator == "btpg":
                label = f"{simulator}-{subplot_key}"
            else:
                label = f"{cycle}-{subplot_key}"
        elif self.plot_type == "category":
            simulator = "online"
            (subplot_key) = indexes
            subplot_key = self.get_subplot_key(subplot_key)
            label = f"{subplot_key}"
        else:
            (simulator, subplot_key) = indexes
            subplot_key = self.get_subplot_key(subplot_key)
            if simulator == "pibt":
                label = f"causal-pibt+-{subplot_key}"
            elif simulator == "default":
                label = f"baseline-{subplot_key}"
            elif simulator == "online":
                label = f"proposed-{subplot_key}"
            elif simulator == "fixed":
                label = f"determined-{subplot_key}"
            elif simulator == "physical":
                label = f"unsettled-physical-{subplot_key}"
            elif simulator == "graph":
                label = f"unsettled-graph-{subplot_key}"
            else:
                label = f"{simulator}-{subplot_key}"

        if self.subplot_type == "delay-ratio":
            label += "-obstacles"
        elif self.subplot_type in ("obstacles", "delay-interval", "map-names", "k_neighbor"):
            label = "-".join(label.split('-')[:-1])

        if self.plot_type == "simulator" or self.plot_type == "cdf":
            if simulator == "default":
                linestyle = "dashdot"
            elif simulator.startswith("replan"):
                linestyle = "dotted"
            elif simulator == "pibt":
                linestyle = (0, (3, 5, 1, 5))  # "dashdotted"
            elif simulator == "prioritized":
                linestyle = "dashed"
            elif simulator == "btpg":
                linestyle = (0, (3, 1, 1, 1, 1, 1))
            else:
                linestyle = "solid"
            # linestyle = simulator == "online" and "-" or (cycle == "naive" and ":" or "-.")
        elif self.plot_type == "replan":
            if simulator == "replan":
                linestyle = "dotted"
            elif simulator == "prioritized":
                linestyle = "dashed"
            else:
                linestyle = "solid"
        elif self.plot_type == "feasibility":
            linestyle = simulator == "heuristic" and "-" or ":"
        elif self.plot_type == "cycle":
            linestyle = simulator == "proposed" and "-" or (simulator == "naive" and "-." or ":")
        elif self.plot_type == "category":
            linestyle = "-"
        elif self.plot_type == "redundant":
            if simulator == "fixed":
                linestyle = "solid"
            elif simulator == "physical":
                linestyle = "dotted"
            elif simulator == "graph":
                linestyle = "dashed"
            else:
                linestyle = "dashdot"
        else:
            assert False

        label = f"\\texttt{{{label}}}"

        return LineSettings(simulator=simulator, label=label, linestyle=linestyle)

    def get_line_values(self, df: pd.DataFrame):
        x = np.arange(len(df))
        y_lower = None
        y_upper = None
        if self.y_field == "time":
            y = np.array(df["average_timestep_time"] * 1000)
            y_lower = y - np.array(df["average_timestep_time_lower"] * 1000)
            y_upper = np.array(df["average_timestep_time_upper"] * 1000) - y
        elif self.y_field == "makespan":
            y = np.array(df["makespan"])
            y_lower = y - np.array(df["makespan_lower"])
            y_upper = np.array(df["makespan_upper"]) - y
        elif self.y_field == "average_cost":
            y = np.array(df["average_cost"] * df["agents"])
            y_lower = y - np.array(df["average_cost_lower"] * df["agents"])
            y_upper = np.array(df["average_cost_upper"] * df["agents"]) - y
        elif self.y_field == "makespan_time":
            y = np.array(df["makespan_time"] * 1000)
            y_lower = y - np.array(df["makespan_time_lower"] * 1000)
            y_upper = np.array(df["makespan_time_upper"] * 1000) - y
        elif self.y_field == "plan_percent":
            y = np.array(df["full_replan_count"] / df["makespan"] * 100)
        elif self.y_field == "node_pair":
            y = np.array(df["node_pair"])
        elif self.y_field == "cdf" or self.y_field == "pdf":
            data = ast.literal_eval(df["data"].iloc[0])
            data = np.array(data).transpose()
            # logger.info(data.shape)
            if self.y_field == "cdf":
                x = data[0]
                y = data[2] * 100
                # logger.info("{}", x[:10])
                # logger.info("{}", y[:10])
            else:
                n = 100000
                x = data[0]
                y = data[1]
                y_int = np.dot(data[0], data[1])
                print(y_int)
                print(sum(y))
                h, e = np.histogram(x, bins=100, weights=y)
                kde = stats.gaussian_kde(x, weights=y)
                # resamples = np.random.choice((e[:-1] + e[1:]) / 2, size=n * 5, p=h / h.sum())
                # print(np.mean(resamples))
                # rkde = stats.gaussian_kde(resamples)
                x = np.linspace(x.min(), x.max())
                y = kde.pdf(x)

        else:
            assert False
        # print(x, y, y_lower, y_upper)
        return x, y, y_lower, y_upper

    def get_output_file(self):
        if self.extra:
            extra = f"-{self.extra}"
        else:
            extra = ""
        return self.args.plot_dir / \
            f"{self.args.timing}-{self.args.map}-{self.plot_type}-{self.subplot_type}-{self.agents}-{self.plot_value}-{self.y_field}{extra}.pdf"


def plot(df: pd.DataFrame, settings: PlotSettings):
    fig = plt.figure(figsize=(16, 3), dpi=100)
    # plt.rcParams.update({'font.size': 16, 'font.family': 'cmss10', 'font.weight': 'bold'})
    plt.rcParams.update({'font.size': 20, "text.usetex": True})
    axes = []

    for i, key in enumerate(settings.subplot_keys):
        ax = plt.subplot(1, len(settings.subplot_keys), i + 1)
        axes.append(ax)
        sub_df = df[df[settings.subplot_field] == key]
        xticks = []
        xmax = 0
        xmin = np.inf
        # print(sub_df)
        if settings.plot_type == "cdf":
            for j, (indexes, df2) in enumerate(sub_df.groupby(settings.groupby)):
                x, y, y_lower, y_upper = settings.get_line_values(df2)
                xmax = max(xmax, np.max(x))
                simulator = indexes[0]
                if simulator != "online":
                    xmin = min(xmin, np.min(x))
        for j, (indexes, df2) in enumerate(sub_df.groupby(settings.groupby)):
            line_settings = settings.get_line_settings(indexes)
            x, y, y_lower, y_upper = settings.get_line_values(df2)

            if settings.plot_type != "cdf":
                if not xticks:
                    for _, row in df2.iterrows():
                        xticks.append(int(row[settings.x_field]))
            else:
                # logger.info("{} {}", x, y)
                index = np.argmax(y >= 99)
                # print(indexes, f"k={key}", x[index])
                simulator = indexes[0]
                if simulator == "online":
                    index = np.argmax(x > xmin)
                    if index > 0:
                        x = np.concatenate(([xmin], x[index:]))
                        y = np.concatenate(([0], y[index:]))
            # else:
            #     x = np.append(x, xmax)
            #     y = np.append(y, 100)
            # print(plot_type, simulator, linestyle, i, j)
            if settings.subplot_type == "obstacles":
                max_lines = len(settings.args.delay_intervals)
            else:
                max_lines = len(settings.args.obstacles)
            max_lines = len(LINE_COLORS)

            # if settings.plot_type == "simulator" and settings.y_field in ("time", "makespan_time"):
            #     line_index = j + 1
            # else:
            #     line_index = j
            line_index = j
            line_settings.color = LINE_COLORS[line_index % max_lines]
            line_settings.marker = LINE_MARKERS[line_index % max_lines]

            if settings.plot_type == "cdf":
                ax.plot(x, y,
                        linestyle=line_settings.linestyle, color=line_settings.color,
                        marker=None, label=line_settings.label,
                        linewidth=1.5)

            elif y_lower is None or y_upper is None:
                ax.plot(xticks, y,
                        linestyle=line_settings.linestyle, color=line_settings.color,
                        marker=line_settings.marker, label=line_settings.label,
                        linewidth=1.5, markersize=2.5)
            else:
                ax.errorbar(xticks, y, yerr=[y_lower, y_upper],
                            linestyle=line_settings.linestyle, color=line_settings.color,
                            marker=line_settings.marker, label=line_settings.label,
                            linewidth=1.5, markersize=2.5, capsize=3)

        if settings.plot_type != "cdf":
            ax.set_xticks(xticks)
        else:
            ax.set_xscale("log")
            # ax.set_xlim(left=1e-7)
            # ax.set_xlim(left=1e-7, right=xmax)
        # ax.set_xticks(np.arange(len(xticks)))
        # ax.set_xticklabels(xticks)
        ax.set_xlabel(settings.x_label, fontsize=22)
        if settings.subplot_type == "delay-ratio":
            title = f"{int(key * 100)}\\% of agents blocked"
        elif settings.subplot_type == "delay-interval":
            title = f"k = {int(key)}"
        elif settings.subplot_type == "obstacles":
            title = f"{int(key / 9)}\\% obstacles"
        elif settings.subplot_type == "map-names":
            if key == "random":
                title = "random-grid"
            else:
                title = key
        elif settings.subplot_type == "k_neighbor":
            title = f"$2^{key}$ connected"
        else:
            title = ""
        ax.set_title(title, fontsize=26)
        if settings.y_log:
            ax.set_yscale("log")
            # ax.set_ylim(bottom=1)
            ax.tick_params(axis='y', which='minor')
            ax.yaxis.set_minor_locator(LogLocator(base=10, subs=[1.0, 0.3], numticks=20))
            ax.yaxis.set_minor_formatter(format_log)
            ax.yaxis.set_major_formatter(format_log)

            # ax.yaxis.set_minor_formatter(LogFormatter(labelOnlyBase=False))
            # ax.yaxis.set_major_formatter(LogFormatter(labelOnlyBase=True))

        if settings.y_field == "time":
            ax.set_ylim(bottom=0)

    # plt.ylabel(ylabel)
    ax = axes[0]
    ax.set_ylabel(settings.y_label, fontsize=26)
    bbox_extra_artists = []
    if settings.legend:
        ax = axes[-2]
        # ax.set_ylabel(ylabel)
        handles, labels = ax.get_legend_handles_labels()
        if settings.subplot_type in ("obstacles", "delay-interval", "map-names"):
            ncol = 4
        elif len(handles) % 3 == 0:
            ncol = 3
            # handles = np.concatenate((handles[::3], handles[1::3], handles[2::3]), axis=0)
            # labels = np.concatenate((labels[::3], labels[1::3], labels[2::3]), axis=0)
        else:
            ncol = 2
        bbox_to_anchor_y = 1.1 + 0.2 * (len(handles) / ncol)
        legend = ax.legend(handles, labels, loc='upper center', ncol=ncol, columnspacing=0.5,
                           bbox_to_anchor=(0.5, bbox_to_anchor_y), bbox_transform=fig.transFigure, fontsize=26)
        bbox_extra_artists.append(legend)

    output_file = settings.get_output_file()
    print(output_file)
    fig.savefig(fname=output_file, bbox_extra_artists=bbox_extra_artists, bbox_inches='tight')
    plt.close()


async def plot_simulator_discrete(args: PlotArguments, agents: int, delay_ratio: float):
    k_neighbor = 2
    filter = {
        "setup.agents": agents,
        "setup.delay_ratio": delay_ratio,
        "setup.k_neighbor": k_neighbor,
    }
    results = []
    async for document in parsed_collection.find(filter):
        result = {
            **document["setup"],
            **document["result"],
        }
        results.append(result)

    df = pd.DataFrame(results)

    groupby = ["simulator", "cycle", "delay_ratio"]
    plot_type = "simulator"
    if args.map == "random":
        subplot_type = "obstacles"
    elif args.map == "mapf":
        # df = df[(df["obstacles"] == 0) | (df["obstacles"] == 270)]
        subplot_type = "map-names"
    else:
        assert False
    # print(df[['simulator', 'soc']])
    # df['soc_mul'] = df['soc'] * 40
    # print(df)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 plot_value=str(delay_ratio), y_field="average_cost", groupby=groupby, legend=True)
    plot(df, plot_settings)
    # plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
    #                              plot_value=str(delay_ratio), y_field="time", groupby=groupby, legend=True)
    # plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents, y_log=True,
                                 plot_value=str(delay_ratio), y_field="makespan_time", groupby=groupby, legend=True)
    plot(df, plot_settings)

    # df = df[(df["simulator"] == "replan_1.1")]
    # plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
    #                              plot_value=str(delay_ratio), y_field="plan_percent", groupby=groupby, legend=True)
    # plot(df, plot_settings)


def plot_simulator(args: PlotArguments, data: pd.DataFrame, agents: int, k_neighbor: int):
    # df = data[
    #     (((data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["cycle"] == "proposed")) |
    #      (data["simulator"] == "default") | (data["simulator"] == "replan") | (data["simulator"] == "pibt") |
    #      (data["simulator"] == "prioritized") | (data["simulator"] == "snapshot") |
    #      (data["simulator"] == "online_remove_redundant") |
    #      (data["simulator"] == "snapshot_end"))
    #     & (data["agents"] == agents) & (data["k_neighbor"] == k_neighbor)]

    df = data[
        ((data["simulator"] == "online_remove_redundant") | (data["simulator"] == "snapshot_end"))
        & (data["agents"] == agents) & (data["k_neighbor"] == k_neighbor) & (data["delay_ratio"] == 0.1)]

    df2 = data[
        ((data["simulator"] == "online") | (data["simulator"] == "online_remove_redundant"))
        & (data["agents"] == agents) & (data["k_neighbor"] == k_neighbor) & (data["delay_ratio"] == 0.1)]

    groupby = ["simulator", "cycle", "delay_ratio"]
    plot_type = "simulator"
    if args.map == "random":
        subplot_type = "obstacles"
    elif args.map == "den520d":
        subplot_type = "map-names"
    else:
        assert False
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 plot_value=str(k_neighbor), y_field="average_cost", groupby=groupby, legend=True)
    plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 plot_value=str(k_neighbor), y_field="time", groupby=groupby, legend=True)
    plot(df2, plot_settings)
    # plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
    #                              k_neighbor=k_neighbor, y_field="makespan_time", groupby=groupby, legend=True)
    # plot(df2, plot_settings)


def plot_simulator_2(args: PlotArguments, data: pd.DataFrame, agents: int, obstacles: int):
    # df = data[
    #     (((data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["cycle"] == "proposed")) |
    #      (data["simulator"] == "default") | (data["simulator"] == "replan") | (data["simulator"] == "pibt") |
    #      (data["simulator"] == "prioritized") | (data["simulator"] == "snapshot") |
    #      (data["simulator"] == "online_remove_redundant") |
    #      (data["simulator"] == "snapshot_end"))
    #     & (data["agents"] == agents) & (data["k_neighbor"] == k_neighbor)]

    df = data[
        ((data["simulator"] == "online_remove_redundant_physical") |
         (data["simulator"] == "snapshot_end"))
        & (data["agents"] == agents) & (data["obstacles"] == obstacles) & (data["delay_ratio"] == 0.1)]

    df2 = data[
        ((data["simulator"] == "online") |
         (data["simulator"] == "online_remove_redundant_physical") |
         (data["simulator"] == "online_remove_redundant_graph"))
        & (data["agents"] == agents) & (data["obstacles"] == obstacles) & (data["delay_ratio"] == 0.1)]

    groupby = ["simulator", "cycle", "delay_ratio"]
    plot_type = "simulator"
    subplot_type = "k_neighbor"
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 plot_value=str(obstacles), y_field="average_cost", groupby=groupby, legend=True)
    plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 plot_value=str(obstacles), y_field="time", groupby=groupby, legend=True)
    plot(df2, plot_settings)
    # plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
    #                              k_neighbor=k_neighbor, y_field="makespan_time", groupby=groupby, legend=True)
    # plot(df2, plot_settings)


def plot_redundant(args: PlotArguments, data: pd.DataFrame, agents: int):
    df = data[
        ((data["simulator"] == "online_remove_redundant_physical") |
         (data["simulator"] == "online_remove_redundant_graph"))
        & (data["agents"] == agents) & (data["delay_ratio"] == 0.1)]
    # print(df)

    dfs = []

    for indexes, df2 in df.groupby(["obstacles", "k_neighbor"]):
        # print(indexes, df2)
        all = df2["all_node_pairs"].iloc[0]
        fixed = df2["fixed_node_pairs"].iloc[0]
        physical = df2[df2["simulator"] == "online_remove_redundant_physical"]["added_node_pairs"].iloc[0]
        graph = df2[df2["simulator"] == "online_remove_redundant_graph"]["added_node_pairs"].iloc[0]

        row = df2.copy().drop(columns=['all_node_pairs', 'fixed_node_pairs', 'added_node_pairs']).iloc[0]
        df3 = pd.DataFrame([row, row, row, row])
        df3["node_pair_type"] = ["all", "fixed", "physical", "graph"]
        df3["node_pair"] = [all, fixed, physical, graph]
        dfs.append(df3)

        # print(df3)
        # print(all, fixed, physical, graph)

    new_df = pd.concat(dfs).reset_index()
    # print(new_df)

    groupby = ["node_pair_type", "obstacles"]
    plot_type = "redundant"
    subplot_type = "obstacles"
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 plot_value="", y_field="node_pair", groupby=groupby, legend=True)
    plot(new_df, plot_settings)


def plot_replan(args: PlotArguments, data: pd.DataFrame, agents: int):
    df = data[((data["simulator"] == "replan") | (data["simulator"] == "prioritized") | (
            data["simulator"] == "prioritized_opt")) & (data["agents"] == agents)]
    groupby = ["simulator", "obstacles"]
    plot_type = "replan"
    subplot_type = "delay-ratio"
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="average_cost", groupby=groupby, plot_value="2", legend=True)
    plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="time", groupby=groupby, plot_value="2", legend=True)
    plot(df, plot_settings)


def plot_cycle(args: PlotArguments, data: pd.DataFrame, agents: int, delay_ratio: float):
    df = data[
        (data["feasibility"] == "heuristic") & (data["agents"] == agents) & (data["delay_ratio"] == delay_ratio) &
        (data["simulator"] == "online")
        ]
    groupby = ["cycle", "delay_ratio"]
    plot_type = "cycle"
    subplot_type = "obstacles"
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 plot_value=str(delay_ratio), y_field="average_cost", groupby=groupby, legend=True)
    # print(df[['simulator', 'soc']])
    plot(df, plot_settings)
    # plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
    #                              plot_value=str(delay_ratio), y_field="time", groupby=groupby, legend=True)
    # plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 plot_value=str(delay_ratio), y_field="makespan_time", groupby=groupby, legend=True)
    plot(df, plot_settings)


def plot_cdf(args: PlotArguments, data: pd.DataFrame, agents: int, obstacles: int):
    df = data[(data["agents"] == agents) & (data["obstacles"] == obstacles)]
    groupby = ["simulator", "obstacles"]
    plot_type = "cdf"
    subplot_type = "delay-interval"
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="cdf", groupby=groupby, plot_value=2, legend=True, extra=str(obstacles))
    plot(df, plot_settings)


async def plot_replan_pdf(args: PlotArguments, agents: int, delay_ratio: float, delay_interval: int):
    k_neighbor = 2
    filter = {
        "setup.agents": agents,
        "setup.delay_ratio": delay_ratio,
        "setup.delay_interval": delay_interval,
        "setup.k_neighbor": k_neighbor,
        "cdf_data.0": {"$exists": True},
    }
    results = []
    async for document in parsed_collection.find(filter):
        result = {
            **document["setup"],
            "data": json.dumps(document["cdf_data"]),
        }
        results.append(result)

    df = pd.DataFrame(results)

    groupby = ["simulator", "delay_ratio"]
    plot_type = "cdf"
    if args.map == "random":
        subplot_type = "obstacles"
    elif args.map == "mapf":
        df = df[(df["obstacles"] == 0) | (df["obstacles"] == 270)]
        subplot_type = "map-names"
    else:
        assert False
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="cdf", plot_value=str(delay_interval), groupby=groupby, legend=True)
    plot(df, plot_settings)


@app_command("plot")
@click.option("--map-names", type=str, default="random", callback=validate_list(str))
@click.pass_context
@asyncio_wrapper
async def main(ctx, map_names):
    data_dir = project_root / "data"
    args = PlotArguments(
        **ctx.obj.__dict__,
        map_names=map_names,
        plot_dir=project_root / "plot"
    )
    click.echo(args)

    # if args.map == "random":
    #     df_discrete = pd.read_csv(data_dir / f"df_{args.timing}.csv")
    # df_discrete = pd.read_csv(data_dir / f"df_discrete.csv")
    # df_discrete_time = pd.read_csv(data_dir / f"df_{args.timing}_time.csv")
    # else:
    df_discrete_random = pd.read_csv(data_dir / f"df_{args.timing}.csv")
    df_discrete_mapf = pd.read_csv(data_dir / f"df_{args.timing}_mapf.csv")
    # df_discrete = pd.concat([df_discrete_random, df_discrete_mapf])
    df_discrete_mapf_time = pd.read_csv(data_dir / f"df_{args.timing}_time_mapf.csv")

    if args.timing == "discrete":
        if args.map == "random":
            df_discrete = df_discrete_random
            for agents in args.agents:
                # plot_cycle(args, df_discrete, agents, 0.1)
                for delay_ratio in args.delay_ratios:
                    # pass
                    await plot_simulator_discrete(args, agents, delay_ratio)
        else:
            # df_discrete = pd.concat([df_discrete_random, df_discrete_mapf])
            # df_discrete = df_discrete_mapf
            # df_discrete = df_discrete.drop(df_discrete[df_discrete.delay_interval == 0].index)
            # for agents in args.agents:
            #     for delay_ratio in args.delay_ratios:
            #         await plot_simulator_discrete(args, agents, delay_ratio)

            # await plot_replan_pdf(args, 10, 0.1, 1)

            for agents in args.agents:
                for delay_interval in args.delay_intervals:
                    await plot_replan_pdf(args, agents, 0.1, delay_interval)

            # plot_cycle(args, df_discrete, agents)
            # for obstacle in args.obstacles:
            #     plot_cdf(args, df_discrete_time, agents, obstacle)
            # plot_replan(args, df_discrete, agents)
    else:
        if args.map == "random":
            df_discrete = df_discrete_random
            for agents in args.agents:
                for obstacles in args.obstacles:
                    plot_simulator_2(args, df_discrete, agents, obstacles)
                plot_redundant(args, df_discrete, agents)

            # for k_neighbor in args.k_neighbors:
            #     plot_simulator(args, df_discrete, agents, k_neighbor)


if __name__ == '__main__':
    main()
