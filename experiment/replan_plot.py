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
INTERVALS = [1, 10, 20]


def plot(df, agents, obstacles, interval):
    df2 = df[(df["agents"] == agents) & (df["obstacles"] == obstacles) & (df["interval"] == interval)]
    raw_dfs = []
    for rate in AGENT_DELAY_RATIOS:
        raw_dfs.append(df2[df2["rate"] == rate])
    base_df = raw_dfs[0]
    for i in range(1, len(raw_dfs)):
        target_df = raw_dfs[i]
        base_df = base_df[['map_seed', 'agent_seed']]
        base_df = base_df.merge(target_df, on=['map_seed', 'agent_seed'], how='inner')
    base_df = base_df[['map_seed', 'agent_seed']]

    data = []
    for i in range(len(raw_dfs)):
        df = raw_dfs[i]
        df = df.merge(base_df, on=['map_seed', 'agent_seed'], how='inner')
        data.append(df["average_timestep_time"][:50])

    x = npy.arange(50)

    fig = plt.figure(figsize=(16, 9), dpi=100)
    plt.rcParams.update({'font.size': 14, 'font.family': 'monospace'})

    for i in range(len(raw_dfs)):
        plt.scatter(x, data[i], label=str(AGENT_DELAY_RATIOS[i]))

    plt.legend()
    plt.xticks([])
    plt.yscale("log")
    plt.title(f"agents: {agents} obstacles: {obstacles} interval: {interval}")

    output_file = plot_dir / f"replan-check-{agents}-{obstacles}-{interval}-time.pdf"
    print(output_file)
    fig.savefig(fname=output_file)
    plt.close()





def main():
    df_periodic = pandas.read_csv(os.path.join(data_dir, "df_replan_periodic.csv"))
    for agent in AGENTS:
        for obstacle in OBSTACLES:
            for interval in INTERVALS:
                plot(df_periodic, agent, obstacle, interval)
                # exit(0)




if __name__ == '__main__':
    main()
