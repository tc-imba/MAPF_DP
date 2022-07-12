import numpy as np
from scipy.stats import gaussian_kde, norm
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

obstacles_color = {
    90: "#5BB8D7",
    180: "#57A86B",
    270: "#A8A857",
}
obstacles_marker = {
    90: "o",
    180: "s",
    270: "^",
}


def plot(df, agents, yfield, groupby, data_type, plot_type, legend=True):
    ylog = False
    if yfield == "time":
        if plot_type == "feasibility":
            ylabel = 'Average Computation Time of Each Feasibility Check (ms)'
        elif plot_type == "cycle":
            ylabel = 'Average Computation Time of Each Timestep (ms)'
            ylog = True
        else:
            assert False
    elif yfield == "value":
        if data_type == "infinite":
            ylabel = 'Success Rate (%)'
        elif data_type == "periodic":
            ylabel = 'Sum of Cost (average)'
        else:
            assert False
    elif yfield == "category":
        ylabel = 'Percentage of Feasibility Category A'
    else:
        assert False

    if data_type == "infinite":
        xlabel = 'timestep (t)'
        xticks_field = 'timestep'
    elif data_type == "periodic":
        xlabel = 'interval (t)'
        xticks_field = 'interval'
    else:
        assert False

    fig = plt.figure(figsize=(16, 9), dpi=100)
    plt.rcParams.update({'font.size': 16, 'font.family': 'monospace'})

    for i, rate in enumerate([0.2, 0.4]):
        ax = plt.subplot(1, 2, i + 1)
        sub_df = df[df["rate"] == rate]
        xticks = []
        for group, df2 in sub_df.groupby(groupby):
            if plot_type == "online-offline":
                (simulator, cycle, obstacles) = group
                obstacles = int(obstacles)
                if cycle != "proposed":
                    cycle = "naive"
                if simulator == "default":
                    label = f"offline-{obstacles}"
                else:
                    label = f"online-{cycle}-{obstacles}"
            elif plot_type == "category":
                obstacles = int(group)
                label = f"{obstacles}"
            else:
                (simulator, obstacles) = group
                obstacles = int(obstacles)
                label = f"{simulator}-{obstacles}"
            if not xticks:
                for _, row in df2.iterrows():
                    xticks.append(f"{int(row[xticks_field])}")
            x = npy.arange(len(df2))
            if yfield == "value":
                if data_type == "infinite":
                    y = npy.array(df2["value"] / df2["agents"] * 100)
                else:
                    y = npy.array(df2["value"])
            elif yfield == "time":
                if plot_type == "feasibility":
                    y = npy.array(df2["execution_time"] / df2["feasibility_count"] * 1000)
                elif plot_type == "cycle":
                    y = npy.array(df2["execution_time"] / df2["cycle_count"] * 1000)
                else:
                    assert False
            elif yfield == "category":
                y = npy.array(df2["feasibility_type_a"] * 100)
            else:
                assert False

            if plot_type == "online-offline":
                linestyle = simulator == "online" and "-" or (cycle == "naive" and ":" or "-.")
            elif plot_type == "feasibility":
                linestyle = simulator == "heuristic" and "-" or ":"
            elif plot_type == "cycle":
                linestyle = simulator == "proposed" and "-" or (simulator == "naive" and ":" or "-.")
            elif plot_type == "category":
                linestyle = "-"
            else:
                assert False

            color = obstacles_color[obstacles]
            marker = obstacles_marker[obstacles]
            ax.plot(x, y, linestyle=linestyle, color=color, marker=marker, label=label,
                    linewidth=2.5, markersize=8)
        ax.set_xticks(npy.arange(len(xticks)), xticks)
        ax.set_xlabel(xlabel)
        ax.set_title(f"{int(rate * 100)}% of agents blocked")
        if ylog:
            ax.set_yscale("log")
            plt.tick_params(axis='y', which='minor')
            ax.yaxis.set_minor_locator(LogLocator(base=10,subs=[2.0,5.0]))
            ax.yaxis.set_minor_formatter(ScalarFormatter())
            ax.yaxis.set_major_formatter(ScalarFormatter())

    plt.ylabel(ylabel)
    bbox_extra_artists = []
    if legend:
        ax = plt.subplot(1, 2, 1)
        handles, labels = ax.get_legend_handles_labels()
        if len(handles) % 3 == 0:
            ncol = 3
            handles = np.concatenate((handles[::3], handles[1::3], handles[2::3]), axis=0)
            labels = np.concatenate((labels[::3], labels[1::3], labels[2::3]), axis=0)
        else:
            ncol = 2
        bbox_to_anchor_y = 0.97 + 0.03 * (len(handles) / ncol)
        legend = ax.legend(handles, labels, loc='upper center', ncol=ncol, columnspacing=0.5,
                           bbox_to_anchor=(0.5, bbox_to_anchor_y), bbox_transform=fig.transFigure)
        bbox_extra_artists.append(legend)

    output_file = plot_dir / f"{plot_type}-{data_type}-{agents}-{yfield}.pdf"
    print(output_file)
    fig.savefig(fname=output_file, bbox_extra_artists=bbox_extra_artists, bbox_inches='tight')
    plt.close()


def generate_table(df, agents, yfield, groupby, data_type, plot_type):
    output_file = plot_dir / f"{plot_type}-{data_type}-{agents}-{yfield}.tex"
    print(output_file)
    with output_file.open('w') as f:
        f.write('\\begin{tabular}{cccccc}\n')
        f.write('Obstacles & $t$ & Blocked \\% & Type A \\% & Type B \\% & Type C \\% \\\\\\hline\n')
        for index, row in df.iterrows():
            obstacles = int(row['obstacles'])
            timestep = int(row['timestep'])
            rate = int(row['rate'] * 100)
            type_a = row['feasibility_type_a'] * 100
            type_b = row['feasibility_type_b'] * 100
            type_c = row['feasibility_type_c'] * 100
            f.write(f"{obstacles} & {timestep} & {rate} & {type_a:.3f} & {type_b:.3f} & {type_c:.3f} \\\\\n")
        f.write('\\end{tabular}')


def plot_online_offline(data, agents, data_type):
    df = data[(((data["feasibility"] == "heuristic") & (data["cycle"] == "proposed")) | (
            data["simulator"] == "default")) & (data["agents"] == agents)]
    groupby = ["simulator", "cycle", "obstacles"]
    plot_type = "online-offline"
    plot(df, agents, "value", groupby, data_type, plot_type)


def plot_feasibility(data, agents, data_type):
    df = data[(data["simulator"] == "online") & (data["cycle"] == "semi-naive") & (data["agents"] == agents)]
    groupby = ["feasibility", "obstacles"]
    plot_type = "feasibility"
    plot(df, agents, "value", groupby, data_type, plot_type)
    plot(df, agents, "time", groupby, data_type, plot_type)


def plot_cycle(data, agents, data_type):
    df = data[(data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["agents"] == agents)]
    groupby = ["cycle", "obstacles"]
    plot_type = "cycle"
    plot(df, agents, "value", groupby, data_type, plot_type, False)
    plot(df, agents, "time", groupby, data_type, plot_type)


def plot_category(data, agents, data_type):
    df = data[(data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["agents"] == agents)]
    groupby = ["obstacles"]
    plot_type = "category"
    plot(df, agents, "category", groupby, data_type, plot_type)
    generate_table(df, agents, "category", groupby, data_type, plot_type)


def main():
    df_infinite = pandas.read_csv(os.path.join(data_dir, "df_infinite.csv"))
    df_periodic = pandas.read_csv(os.path.join(data_dir, "df_periodic.csv"))
    df_infinite_feasibility_category = pandas.read_csv(os.path.join(data_dir, "df_infinite_feasibility_category.csv"))
    for agents in [10, 20]:
        # plot_online_offline(df_infinite, agents, "infinite")
        # plot_online_offline(df_periodic, agents, "periodic")
        # plot_feasibility(df_infinite, agents, "infinite")
        # plot_feasibility(df_periodic, agents, "periodic")
        # plot_cycle(df_infinite, agents, "infinite")
        # plot_cycle(df_periodic, agents, "periodic")
        plot_category(df_infinite_feasibility_category, agents, "infinite")


if __name__ == '__main__':
    main()
