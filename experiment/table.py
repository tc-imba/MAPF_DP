import numpy as np
from scipy.stats import gaussian_kde, norm
import numpy as npy
import matplotlib.pyplot as plt
import pandas

import os
from pprint import pprint

project_root = os.path.dirname(os.path.dirname(__file__))
result_dir = os.path.join(project_root, "result")
plot_dir = os.path.join(project_root, "plot")
data_dir = os.path.join(project_root, "data")
os.makedirs(plot_dir, exist_ok=True)

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


def plot(df, agents, yfield, groupby, data_type, plot_type):
    ylog = False
    if yfield == "time":
        if plot_type == "feasibility":
            ylabel = 'Computation Time of Each Feasibility Check (milliseconds)'
        elif plot_type == "cycle":
            ylabel = 'Computation Time of Each Cycle Check (milliseconds)'
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
                y = npy.array(df2["execution_time"] / df2["feasibility_count"] * 1000)

            if plot_type == "online-offline":
                linestyle = simulator == "online" and "-" or (cycle == "naive" and ":" or "-.")
            elif plot_type == "feasibility":
                linestyle = simulator == "heuristic" and "-" or ":"
            elif plot_type == "cycle":
                linestyle = simulator == "proposed" and "-" or (simulator == "naive" and ":" or "-.")
            else:
                assert False

            color = obstacles_color[obstacles]
            marker = obstacles_marker[obstacles]
            ax.plot(x, y, linestyle=linestyle, color=color, marker=marker, label=label,
                    linewidth=2.5, markersize=8)
        ax.set_xticks(npy.arange(len(xticks)), xticks)
        ax.set_xlabel(xlabel)
        ax.set_title(f"{int(rate * 100)}% of agents blocked")
        # if ylog:
        #     ax.set_yscale("log")

    plt.legend()
    plt.ylabel(ylabel)
    plt.tight_layout()
    output_file = os.path.join(plot_dir, f"{plot_type}-{data_type}-{agents}-{yfield}.png")
    print(output_file)
    fig.savefig(fname=output_file, dpi=300)
    plt.close()


def plot_online_offline(data, agents, data_type):
    df = data[(((data["feasibility"] == "heuristic") & (data["cycle"] != "naive")) | (
            data["simulator"] == "default")) & (data["agents"] == agents)]
    groupby = ["simulator", "cycle", "obstacles"]
    plot_type = "online-offline"
    plot(df, agents, "value", groupby, data_type, plot_type)


def plot_feasibility(data, agents, data_type):
    df = data[(data["simulator"] == "online") & (data["cycle"] == "naive (only cycle)") & (data["agents"] == agents)]
    groupby = ["feasibility", "obstacles"]
    plot_type = "feasibility"
    plot(df, agents, "value", groupby, data_type, plot_type)
    plot(df, agents, "time", groupby, data_type, plot_type)


def plot_cycle(data, agents, data_type):
    df = data[(data["simulator"] == "online") & (data["feasibility"] == "heuristic") & (data["agents"] == agents)]
    groupby = ["cycle", "obstacles"]
    plot_type = "cycle"
    plot(df, agents, "value", groupby, data_type, plot_type)
    plot(df, agents, "time", groupby, data_type, plot_type)


def main():
    df_infinite = pandas.read_csv(os.path.join(data_dir, "df_infinite.csv"))
    df_periodic = pandas.read_csv(os.path.join(data_dir, "df_periodic.csv"))
    for agents in [10, 20]:
        plot_online_offline(df_infinite, agents, "infinite")
        plot_online_offline(df_periodic, agents, "periodic")
        plot_feasibility(df_infinite, agents, "infinite")
        plot_feasibility(df_periodic, agents, "periodic")
        plot_cycle(df_infinite, agents, "infinite")
        plot_cycle(df_periodic, agents, "periodic")


if __name__ == '__main__':
    main()
