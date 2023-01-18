import numpy as npy
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, LogLocator
import pandas
from pathlib import Path

import os
from pprint import pprint

project_root = Path(__file__).parent.parent
result_dir = project_root / "result"
plot_dir = project_root / "plot"
data_dir = project_root / "data"
plot_dir.mkdir(exist_ok=True)

obstacles_color = ["#5BB8D7", "#57A86B", "#A8A857"]
obstacles_marker = ["o", "s", "^"]

EDGE_DELAY_RATIOS = [0.01, 0.05]
AGENT_DELAY_RATIOS = [0.1, 0.2, 0.3]
OBSTACLES = [90, 270]
AGENTS = [20, 30]


def get_subplot_key(subplot_type, data):
    if subplot_type == "delay-ratio":
        subplot_key = int(data)
    elif subplot_type == "obstacle":
        subplot_key = f"{int(data * 100)}%"
    else:
        assert False
    return subplot_key


def plot(df, agents, yfield, groupby, data_type, plot_type, delay_type, subplot_type, legend=True):
    ylog = False
    if yfield == "time":
        if plot_type == "simulator":
            ylabel = 'Average Computation Time \n of Each Timestep (ms)'
        elif plot_type == "feasibility":
            ylabel = 'Average Computation Time of Each Feasibility Check (ms)'
        elif plot_type == "cycle":
            ylabel = 'Average Computation Time \n of Each Timestep (ms)'
            ylog = True
        else:
            assert False
    elif yfield == "value":
        if data_type == "infinite":
            ylabel = 'Success Rate (%)'
        elif data_type == "periodic":
            ylabel = '\n Sum of Costs'
        else:
            assert False
    elif yfield == "loop":
        ylabel = 'Edges added into Stack in Feasibility Check'
    elif yfield == "category":
        ylabel = 'Percentage of Feasibility Category A'
    else:
        assert False

    if data_type == "infinite":
        xlabel = 'start timestep (t)'
        xticks_field = 'timestep'
    elif data_type == "periodic":
        xlabel = 'delay interval (k)'
        xticks_field = 'interval'
    else:
        assert False

    fig = plt.figure(figsize=(16, 3), dpi=100)
    plt.rcParams.update({'font.size': 14, 'font.family': 'monospace'})
    axes = []

    if subplot_type == "delay-ratio":
        subplot_field = "rate"
        if delay_type == "edge":
            subplot_keys = EDGE_DELAY_RATIOS
        elif delay_type == "agent":
            subplot_keys = AGENT_DELAY_RATIOS
        else:
            assert False
    elif subplot_type == "obstacle":
        subplot_field = "obstacles"
        subplot_keys = OBSTACLES
    else:
        assert False

    for i, key in enumerate(subplot_keys):
        ax = plt.subplot(1, len(subplot_keys), i + 1)
        axes.append(ax)
        sub_df = df[df[subplot_field] == key]
        xticks = []
        for j, (group, df2) in enumerate(sub_df.groupby(groupby)):
            if plot_type == "simulator":
                (simulator, cycle, subplot_key) = group
                subplot_key = get_subplot_key(subplot_type, subplot_key)
                if cycle != "proposed":
                    cycle = "naive"
                if simulator == "default":
                    label = f"baseline-{subplot_key}"
                elif simulator == "replan":
                    label = f"replan-{subplot_key}"
                else:
                    label = f"{cycle}-{subplot_key}"
            elif plot_type == "category":
                subplot_key = get_subplot_key(subplot_type, group)
                label = f"{subplot_key}"
            else:
                (simulator, subplot_key) = group
                subplot_key = get_subplot_key(subplot_type, subplot_key)
                label = f"{simulator}-{subplot_key}"

            if subplot_type == "delay-ratio":
                label += "-obstacles"
            elif subplot_type == "obstacle":
                label += "-agents-paused"

            if not xticks:
                for _, row in df2.iterrows():
                    xticks.append(f"{int(row[xticks_field])}")
            x = npy.arange(len(df2))
            if yfield == "value":
                if data_type == "infinite":
                    y = npy.array(df2["value"] / df2["agents"] * 100)
                else:
                    y = npy.array(df2["value"] * df2["agents"])
            elif yfield == "time":
                y = npy.array(df2["execution_time"] / df2["first_agent_arriving"] * 1000)
                # if plot_type == "feasibility":
                #     y = npy.array(df2["execution_time"] / df2["first_agent_arriving"] * 1000)
                # elif plot_type == "cycle":
                #     y = npy.array(df2["execution_time"] / df2["first_agent_arriving"] * 1000)
                # else:
                #     assert False
            elif yfield == "loop":
                y = npy.array(df2["average_feasibility_loop"])
            elif yfield == "category":
                y = npy.array(df2["feasibility_type_a"] * 100)
            else:
                assert False

            if plot_type == "simulator":
                if simulator == "default":
                    linestyle = "-."
                elif simulator == "replan":
                    linestyle = ":"
                else:
                    linestyle = "-"
                # linestyle = simulator == "online" and "-" or (cycle == "naive" and ":" or "-.")
            elif plot_type == "feasibility":
                linestyle = simulator == "heuristic" and "-" or ":"
            elif plot_type == "cycle":
                linestyle = simulator == "proposed" and "-" or (simulator == "naive" and ":" or "-.")
            elif plot_type == "category":
                linestyle = "-"
            else:
                assert False
            # print(plot_type, simulator, linestyle, i, j)
            color = obstacles_color[j % len(OBSTACLES)]
            marker = obstacles_marker[j % len(OBSTACLES)]
            ax.plot(x, y, linestyle=linestyle, color=color, marker=marker, label=label,
                    linewidth=2.5, markersize=8)
        ax.set_xticks(npy.arange(len(xticks)))
        ax.set_xticklabels(xticks)
        ax.set_xlabel(xlabel)
        if subplot_type == "delay-ratio":
            ax.set_title(f"{int(key * 100)}% of {delay_type}s blocked")
        elif subplot_type == "obstacle":
            ax.set_title(f"{int(key)} obstacles")
        if ylog:
            ax.set_yscale("log")
            plt.tick_params(axis='y', which='minor')
            ax.yaxis.set_minor_locator(LogLocator(base=10, subs=[2.0, 5.0]))
            ax.yaxis.set_minor_formatter(ScalarFormatter())
            ax.yaxis.set_major_formatter(ScalarFormatter())

    # plt.ylabel(ylabel)
    ax = axes[0]
    ax.set_ylabel(ylabel)
    bbox_extra_artists = []
    if legend:
        # ax = axes[0]
        # ax.set_ylabel(ylabel)
        handles, labels = ax.get_legend_handles_labels()
        if len(handles) % 3 == 0:
            ncol = 3
            # handles = np.concatenate((handles[::3], handles[1::3], handles[2::3]), axis=0)
            # labels = np.concatenate((labels[::3], labels[1::3], labels[2::3]), axis=0)
        else:
            ncol = 2
        bbox_to_anchor_y = 0.75 + 0.25 * (len(handles) / ncol)
        legend = ax.legend(handles, labels, loc='upper center', ncol=ncol, columnspacing=0.5,
                           bbox_to_anchor=(0.5, bbox_to_anchor_y), bbox_transform=fig.transFigure)
        bbox_extra_artists.append(legend)

    output_file = plot_dir / f"{delay_type}-{plot_type}-{subplot_type}-{data_type}-{agents}-{yfield}.pdf"
    print(output_file)
    fig.savefig(fname=output_file, bbox_extra_artists=bbox_extra_artists, bbox_inches='tight')
    plt.close()


def generate_table(df, agents, yfield, groupby, data_type, plot_type):
    output_file = plot_dir / f"{plot_type}-{data_type}-{agents}-{yfield}.tex"
    print(output_file)
    if data_type == "infinite":
        timestep_label = "t"
    elif data_type == "periodic":
        timestep_label = "k"
    else:
        assert False
    with output_file.open('w') as f:
        f.write('\\begin{tabular}{cccccc}\n')
        f.write(f'Obstacles & ${timestep_label}$ & Blocked \\% & TP \\% & TN \\% & FN \\% \\\\\\hline\n')
        for index, row in df.iterrows():
            obstacles = int(row['obstacles'])
            if data_type == "infinite":
                timestep = int(row['timestep'])
            elif data_type == "periodic":
                timestep = int(row['interval'])
            else:
                assert False
            rate = int(row['rate'] * 100)
            type_a = row['feasibility_type_a'] * 100
            type_b = row['feasibility_type_b'] * 100
            type_c = row['feasibility_type_c'] * 100
            f.write(f"{obstacles} & {timestep} & {rate} & {type_a:.3f} & {type_b:.3f} & {type_c:.3f} \\\\\n")
        f.write('\\end{tabular}')


def plot_simulator(data, agents, data_type, delay_type):
    df = data[(((data["feasibility"] == "heuristic") & (data["cycle"] == "proposed")) | (
            data["simulator"] == "default") | (data["simulator"] == "replan"))
              & (data["agents"] == agents) & (data["delay_type"] == delay_type)]
    df2 = df[df["simulator"] != "default"]
    groupby = ["simulator", "cycle", "obstacles"]
    plot_type = "simulator"
    subplot_type = "delay-ratio"
    plot(df, agents, "value", groupby, data_type, plot_type, delay_type, subplot_type, legend=agents == 20)
    plot(df2, agents, "time", groupby, data_type, plot_type, delay_type, subplot_type, legend=False)
    groupby = ["simulator", "cycle", "rate"]
    subplot_type = "obstacle"
    plot(df, agents, "value", groupby, data_type, plot_type, delay_type, subplot_type, legend=agents == 20)
    plot(df2, agents, "time", groupby, data_type, plot_type, delay_type, subplot_type, legend=False)


def plot_feasibility(data, agents, data_type, delay_type):
    df = data[(data["simulator"] == "online") & (data["cycle"] == "proposed") & (data["agents"] == agents) & (
            data["delay_type"] == delay_type) & (data["obstacles"] == 270)]
    groupby = ["feasibility", "obstacles"]
    plot_type = "feasibility"
    subplot_type = "delay-ratio"
    plot(df, agents, "value", groupby, data_type, plot_type, delay_type, subplot_type)
    plot(df, agents, "time", groupby, data_type, plot_type, delay_type, subplot_type)
    plot(df, agents, "loop", groupby, data_type, plot_type, delay_type, subplot_type)
    groupby = ["feasibility", "rate"]
    subplot_type = "obstacle"
    plot(df, agents, "value", groupby, data_type, plot_type, delay_type, subplot_type)
    plot(df, agents, "time", groupby, data_type, plot_type, delay_type, subplot_type)
    plot(df, agents, "loop", groupby, data_type, plot_type, delay_type, subplot_type)


def plot_cycle(data, agents, data_type, delay_type):
    df = data[(data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["agents"] == agents) & (
            data["delay_type"] == delay_type)]
    groupby = ["cycle", "obstacles"]
    plot_type = "cycle"
    subplot_type = "delay-ratio"
    plot(df, agents, "value", groupby, data_type, plot_type, delay_type, subplot_type, legend=agents == 20)
    plot(df, agents, "time", groupby, data_type, plot_type, delay_type, subplot_type, legend=False)
    groupby = ["cycle", "rate"]
    subplot_type = "obstacle"
    plot(df, agents, "value", groupby, data_type, plot_type, delay_type, subplot_type, legend=agents == 20)
    plot(df, agents, "time", groupby, data_type, plot_type, delay_type, subplot_type, legend=False)


def plot_category(data, agents, data_type, delay_type):
    df = data[(data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["agents"] == agents) & (
            data["delay_type"] == delay_type)]
    groupby = ["obstacles"]
    plot_type = "category"
    subplot_type = "delay-ratio"
    plot(df, agents, "category", groupby, data_type, plot_type, delay_type, subplot_type)
    generate_table(df, agents, "category", groupby, data_type, plot_type)


def main():
    df_infinite = pandas.read_csv(os.path.join(data_dir, "df_infinite.csv"))
    df_periodic = pandas.read_csv(os.path.join(data_dir, "df_periodic.csv"))
    df_infinite_feasibility_category = pandas.read_csv(os.path.join(data_dir, "df_infinite_feasibility_category.csv"))
    df_periodic_feasibility_category = pandas.read_csv(os.path.join(data_dir, "df_periodic_feasibility_category.csv"))
    for delay_type in ["agent"]:
        for agents in AGENTS:
            # plot_simulator(df_infinite, agents, "infinite", delay_type)
            plot_simulator(df_periodic, agents, "periodic", delay_type)
            # plot_feasibility(df_infinite, agents, "infinite", delay_type)
            # plot_feasibility(df_periodic, agents, "periodic", delay_type)
            # plot_cycle(df_infinite, agents, "infinite", delay_type)
            plot_cycle(df_periodic, agents, "periodic", delay_type)
            # plot_category(df_infinite_feasibility_category, agents, "infinite", delay_type)
            # plot_category(df_periodic_feasibility_category, agents, "periodic", delay_type)


if __name__ == '__main__':
    main()
