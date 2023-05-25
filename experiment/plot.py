import dataclasses
from pathlib import Path
from typing import List, Any
import click
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, LogLocator, LogFormatter
import numpy as np
from tqdm import tqdm

from experiment.app import app_command, AppArguments
from experiment.utils import asyncio_wrapper, project_root, ExperimentSetup

LINE_COLORS = ["#5BB8D7", "#57A86B", "#A8A857", "#0a2129"]
LINE_MARKERS = ["o", "s", "^", "+"]


def format_log(x, pos=None):
    x = float(x)
    if x.is_integer():
        return str(int(x))
    return str(x)


@dataclasses.dataclass
class PlotArguments(AppArguments):
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
    y_field: str
    groupby: List[str]

    legend: bool = True

    # automatically initialized
    subplot_field: str = ""
    subplot_keys: List[Any] = dataclasses.field(default_factory=list)
    y_log: bool = False
    y_label: str = ""
    x_field: str = ""
    x_label: str = ""

    def __post_init__(self):
        self.y_log = False
        if self.y_field == "time":
            if self.plot_type == "simulator" or self.plot_type == "replan":
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
        elif self.y_field == "soc":
            self.y_label = '\n Sum of Costs'
        elif self.y_field == "makespan_time":
            self.y_label = 'Average Computation Time \n of Each Timestep (ms)'
            # self.y_log = True
        else:
            assert False

        self.x_label = 'length of each pause (k)'
        self.x_field = 'delay_interval'

        if self.subplot_type == "delay-ratio":
            self.subplot_field = "delay_ratio"
            self.subplot_keys = self.args.delay_ratios
        elif self.subplot_type == "obstacles":
            self.subplot_field = "obstacles"
            self.subplot_keys = self.args.obstacles
        else:
            assert False

    def get_subplot_key(self, subplot_key):
        if self.subplot_type == "delay-ratio":
            subplot_key = int(subplot_key)
        elif self.subplot_type == "obstacles":
            subplot_key = f"{int(subplot_key * 100)}%"
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
            elif simulator == "replan":
                label = f"replan-{subplot_key}"
            elif simulator == "pibt":
                label = f"pibt-{subplot_key}"
            elif simulator == "prioritized":
                label = f"prioritized-{subplot_key}"
            else:
                label = f"{cycle}-{subplot_key}"
        elif self.plot_type == "category":
            simulator = "online"
            subplot_key = indexes
            subplot_key = self.get_subplot_key(subplot_key)
            label = f"{subplot_key}"
        else:
            (simulator, subplot_key) = indexes
            subplot_key = self.get_subplot_key(subplot_key)
            label = f"{simulator}-{subplot_key}"

        if self.subplot_type == "delay-ratio":
            label += "-obstacles"
        elif self.subplot_type == "obstacles":
            label = "-".join(label.split('-')[:-1])

        if self.plot_type == "simulator":
            if simulator == "default":
                linestyle = "dashdot"
            elif simulator == "replan":
                linestyle = "dotted"
            elif simulator == "pibt":
                linestyle = (0, (3, 5, 1, 5))  # "dashdotted"
            elif simulator == "prioritized":
                linestyle = "dashed"
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
        else:
            assert False

        return LineSettings(simulator=simulator, label=label, linestyle=linestyle)

    def get_line_values(self, df: pd.DataFrame):
        x = np.arange(len(df))
        if self.y_field == "time":
            y = np.array(df["average_timestep_time"] * 1000)
            y_lower = y - np.array(df["average_timestep_time_lower"] * 1000)
            y_upper = np.array(df["average_timestep_time_upper"] * 1000) - y
        elif self.y_field == "makespan":
            y = np.array(df["makespan"])
            y_lower = y - np.array(df["makespan_lower"])
            y_upper = np.array(df["makespan_upper"]) - y
        elif self.y_field == "soc":
            y = np.array(df["soc"] * df["agents"])
            y_lower = y - np.array(df["soc_lower"] * df["agents"])
            y_upper = np.array(df["soc_upper"] * df["agents"]) - y
        elif self.y_field == "makespan_time":
            y = np.array(df["makespan_time"] * 1000)
            y_lower = y - np.array(df["makespan_time_lower"] * 1000)
            y_upper = np.array(df["makespan_time_upper"] * 1000) - y
        else:
            assert False
        return x, y, y_lower, y_upper

    def get_output_file(self):
        return self.args.plot_dir / \
            f"{self.plot_type}-{self.subplot_type}-{self.agents}-{self.y_field}.pdf"


def plot(df: pd.DataFrame, settings: PlotSettings):
    fig = plt.figure(figsize=(16, 3), dpi=100)
    plt.rcParams.update({'font.size': 14, 'font.family': 'monospace'})
    axes = []

    for i, key in enumerate(settings.subplot_keys):
        ax = plt.subplot(1, len(settings.subplot_keys), i + 1)
        axes.append(ax)
        sub_df = df[df[settings.subplot_field] == key]
        xticks = []
        for j, (indexes, df2) in enumerate(sub_df.groupby(settings.groupby)):
            line_settings = settings.get_line_settings(indexes)
            if not xticks:
                for _, row in df2.iterrows():
                    xticks.append(int(row[settings.x_field]))
            x, y, y_lower, y_upper = settings.get_line_values(df2)

            # print(plot_type, simulator, linestyle, i, j)
            if settings.subplot_type == "obstacles":
                max_lines = len(settings.args.delay_intervals)
            else:
                max_lines = len(settings.args.obstacles)

            # if settings.plot_type == "simulator" and settings.y_field in ("time", "makespan_time"):
            #     line_index = j + 1
            # else:
            #     line_index = j
            line_index = j
            line_settings.color = LINE_COLORS[line_index % max_lines]
            line_settings.marker = LINE_MARKERS[line_index % max_lines]

            if y_lower is None or y_upper is None:
                ax.plot(xticks, y,
                        linestyle=line_settings.linestyle, color=line_settings.color,
                        marker=line_settings.marker, label=line_settings.label,
                        linewidth=1.5, markersize=2.5)
            else:
                ax.errorbar(xticks, y, yerr=[y_lower, y_upper],
                            linestyle=line_settings.linestyle, color=line_settings.color,
                            marker=line_settings.marker, label=line_settings.label,
                            linewidth=1.5, markersize=2.5, capsize=3)
        ax.set_xticks(xticks)
        # ax.set_xticks(np.arange(len(xticks)))
        # ax.set_xticklabels(xticks)
        ax.set_xlabel(settings.x_label)
        if settings.subplot_type == "delay-ratio":
            ax.set_title(f"{int(key * 100)}% of agents blocked")
        elif settings.subplot_type == "obstacles":
            ax.set_title(f"{int(key)} obstacles")
        if settings.y_log:
            ax.set_yscale("log")
            # ax.set_ylim(bottom=1)
            ax.tick_params(axis='y', which='minor')
            ax.yaxis.set_minor_locator(LogLocator(base=10, subs=[1.0, 0.3], numticks=20))
            ax.yaxis.set_minor_formatter(format_log)
            ax.yaxis.set_major_formatter(format_log)

            # ax.yaxis.set_minor_formatter(LogFormatter(labelOnlyBase=False))
            # ax.yaxis.set_major_formatter(LogFormatter(labelOnlyBase=True))

    # plt.ylabel(ylabel)
    ax = axes[0]
    ax.set_ylabel(settings.y_label)
    bbox_extra_artists = []
    if settings.legend:
        # ax = axes[0]
        # ax.set_ylabel(ylabel)
        handles, labels = ax.get_legend_handles_labels()
        if settings.subplot_type == "obstacles":
            ncol = 4
        elif len(handles) % 3 == 0:
            ncol = 3
            # handles = np.concatenate((handles[::3], handles[1::3], handles[2::3]), axis=0)
            # labels = np.concatenate((labels[::3], labels[1::3], labels[2::3]), axis=0)
        else:
            ncol = 2
        bbox_to_anchor_y = 1.05 + 0.1 * (len(handles) / ncol)
        legend = ax.legend(handles, labels, loc='upper center', ncol=ncol, columnspacing=0.5,
                           bbox_to_anchor=(0.5, bbox_to_anchor_y), bbox_transform=fig.transFigure)
        bbox_extra_artists.append(legend)

    output_file = settings.get_output_file()
    print(output_file)
    fig.savefig(fname=output_file, bbox_extra_artists=bbox_extra_artists, bbox_inches='tight')
    plt.close()


def plot_simulator(args: PlotArguments, data: pd.DataFrame, agents: int):
    df = data[
        (((data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["cycle"] == "proposed")) | (
                data["simulator"] == "default") | (data["simulator"] == "replan") | (data["simulator"] == "pibt") | (
                 data["simulator"] == "prioritized"))
        & (data["agents"] == agents)]
    df2 = df[df["simulator"] != "default"]
    groupby = ["simulator", "cycle", "delay_ratio"]
    plot_type = "simulator"
    subplot_type = "obstacles"
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="soc", groupby=groupby, legend=True)
    plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="time", groupby=groupby, legend=False)
    plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="makespan_time", groupby=groupby, legend=False)
    plot(df, plot_settings)


def plot_replan(args: PlotArguments, data: pd.DataFrame, agents: int):
    df = data[((data["simulator"] == "replan") | (data["simulator"] == "prioritized") | (
            data["simulator"] == "prioritized_opt")) & (data["agents"] == agents)]
    groupby = ["simulator", "obstacles"]
    plot_type = "replan"
    subplot_type = "delay-ratio"
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="soc", groupby=groupby, legend=True)
    plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="time", groupby=groupby, legend=True)
    plot(df, plot_settings)


def plot_cycle(args: PlotArguments, data: pd.DataFrame, agents: int):
    df = data[(data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["agents"] == agents)]
    groupby = ["cycle", "delay_ratio"]
    plot_type = "cycle"
    subplot_type = "obstacles"
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="soc", groupby=groupby, legend=True)
    plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="time", groupby=groupby, legend=False)
    plot(df, plot_settings)
    plot_settings = PlotSettings(args=args, plot_type=plot_type, subplot_type=subplot_type, agents=agents,
                                 y_field="makespan_time", groupby=groupby, legend=False)
    plot(df, plot_settings)


@app_command("plot")
@click.pass_context
def main(ctx):
    data_dir = project_root / "data"
    args = PlotArguments(
        **ctx.obj.__dict__,
        plot_dir=project_root / "plot"
    )
    click.echo(args)

    df_discrete = pd.read_csv(data_dir / "df_discrete.csv")
    for agents in args.agents:
        plot_simulator(args, df_discrete, agents)
        # plot_replan(args, df_discrete, agents)
        plot_cycle(args, df_discrete, agents)


if __name__ == '__main__':
    main()
