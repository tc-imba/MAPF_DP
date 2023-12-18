import numpy as npy
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, LogLocator
import pandas as pd
from pathlib import Path

import os
from pprint import pprint

project_root = Path(__file__).parent.parent
result_dir = project_root / "result"
plot_dir = project_root / "plot"
data_dir = project_root / "data"


def main():
    df_discrete = pd.read_csv(data_dir / "df_discrete.csv")
    df_discrete_old = pd.read_csv(data_dir / "df_discrete_old.csv")

    condition = (df_discrete_old["simulator"] == "default") | (df_discrete_old["simulator"] == "pibt")
    df_discrete_old = df_discrete_old.drop(df_discrete_old[condition].index).reset_index()

    for (agents, obstacles, delay_ratio, delay_interval), group in df_discrete_old[df_discrete_old["simulator"] == "online"].groupby(["agents", "obstacles", "delay_ratio", "delay_interval"]):
        target_row = df_discrete[(df_discrete["simulator"] == "online_opt") & (df_discrete["agents"] == agents) & (df_discrete["obstacles"] == obstacles) & (df_discrete["delay_ratio"] == delay_ratio) & (df_discrete["delay_interval"] == delay_interval)]
        source_row = group[(group["cycle"] == "proposed")]
        # print(source_row)
        # print(target_row)
        for column in ("soc", "soc_lower", "soc_upper"):
            diff = float(target_row[column].iloc[0]) - float(source_row[column].iloc[0])
            for i, row in group.iterrows():
                df_discrete_old.loc[i, column] += diff
        # print(df_discrete_old.index[group])
        # for i, row in group.iterrows():
        #     df_discrete_old.loc[i, "soc"] = float(target_row["soc"].iloc[0])
        #     df_discrete_old.loc[i, "soc_lower"] = float(target_row["soc_lower"].iloc[0])
        #     df_discrete_old.loc[i, "soc_upper"] = float(target_row["soc_upper"].iloc[0])


    # df_discrete_old.rename(columns={
    #     "value": "soc",
    #     "interval": "delay_interval",
    #     "rate": "delay_ratio",
    #     "timestep": "delay_start",
    #     "value_lower": "soc_lower",
    #     "value_upper": "soc_upper",
    #     "time": "total_time",
    # }, inplace=True)
    #
    # df_discrete_old["timing"] = "discrete"
    df_discrete_old["k_neighbor"] = 2
    df_discrete_old["map_name"] = "random"
    df_discrete_old["added_node_pairs"] = 0
    df_discrete_old["all_node_pairs"] = 0
    df_discrete_old["fixed_node_pairs"] = 0
    # df_discrete_old["full_replan_count"] = 0
    # df_discrete_old["full_replan_time"] = 0
    # df_discrete_old["partial_replan_count"] = 0
    # df_discrete_old["partial_replan_time"] = 0
    # df_discrete_old["unblocked_agents"] = 0
    # df_discrete_old["data_points"] = 1000
    # df_discrete_old["makespan_time"] = df_discrete_old["execution_time"]
    # df_discrete_old["makespan_time_lower"] = df_discrete_old["makespan_time"]
    # df_discrete_old["makespan_time_upper"] = df_discrete_old["makespan_time"]
    # df_discrete_old["makespan_lower"] = df_discrete_old["makespan"]
    # df_discrete_old["makespan_upper"] = df_discrete_old["makespan"]


    # print(sorted(df_discrete.columns))
    # print(sorted(df_discrete_old.columns))

    df_combined = pd.concat([df_discrete, df_discrete_old])
    df_combined = df_combined[df_discrete.columns]

    # print(df_combined)

    df_combined.to_csv(data_dir / "df_discrete_combined.csv", index=False)





if __name__ == '__main__':
    main()

