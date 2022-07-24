import numpy as npy
import pandas
import os

project_root = os.path.dirname(os.path.dirname(__file__))
result_dir = os.path.join(project_root, "result")
plot_dir = os.path.join(project_root, "plot")
data_dir = os.path.join(project_root, "data")
os.makedirs(plot_dir, exist_ok=True)
os.makedirs(data_dir, exist_ok=True)

# obstacles_list = [90, 180, 270, 360, 450]
obstacles_list = [90, 180, 270]
agents_list = [10, 20, 30]
simulators_list = ["online", "default"]
delay_ratios_list = [0.1, 0.2]


def parse_data(data_type) -> pandas.DataFrame:
    if data_type == "infinite":
        starts_list = [1, 5, 10]
        intervals_list = [0]
    elif data_type == "periodic":
        starts_list = [1]
        intervals_list = [1, 5, 10]
    else:
        assert False

    main_df = pandas.DataFrame()
    for obstacles in obstacles_list:
        for simulator in simulators_list:
            for agents in agents_list:
                for start in starts_list:
                    for interval in intervals_list:
                        for rate in delay_ratios_list:
                            for feasibility, cycle in [("h", "h"), ("h", "n"), ("n", "h"), ("n", "n"), ("n", "o"),
                                                       ("h", "o")]:
                                # for feasibility, cycle in [("h", "h"), ("n", "h")]:
                                file = f"{simulator}-{obstacles}-{agents}-edge-{rate}-{start}-{interval}-{feasibility}-{cycle}.csv"
                                try:
                                    df = pandas.read_csv(os.path.join(result_dir, file), header=None)
                                    value = npy.mean(df.iloc[:, 0])
                                    time = npy.mean(df.iloc[:, 2])
                                    execution_time = 0
                                    first_agent_arriving = 0
                                    cycle_count = 0
                                    cycle_agents = 0
                                    unblocked_agents = 0
                                    feasibility_count = 0
                                    feasibility_type_a = 0
                                    feasibility_type_b = 0
                                    feasibility_type_c = 0
                                    if len(df.columns) > 8:
                                        execution_time = npy.mean(df.iloc[:, 3])
                                        first_agent_arriving = npy.mean(df.iloc[:, 4])
                                        cycle_count = npy.mean(df.iloc[:, 5])
                                        cycle_agents = npy.mean(df.iloc[:, 6])
                                        unblocked_agents = npy.mean(df.iloc[:, 7])
                                        feasibility_count = npy.mean(df.iloc[:, 8])
                                    if len(df.columns) > 12:
                                        feasibility_count_all = npy.sum(df.iloc[:, 9:13], axis=1)
                                        feasibility_type_a = npy.nanmean(df.iloc[:, 9] / feasibility_count_all)
                                        feasibility_type_b = npy.nanmean(df.iloc[:, 12] / feasibility_count_all)
                                        feasibility_type_c = npy.nanmean((df.iloc[:, 10] + df.iloc[:, 11]) / feasibility_count_all)
                                except:
                                    value = 0
                                    time = 0
                                # print(file, mean)
                                if time == 0:
                                    continue
                                row = {
                                    "simulator": simulator,
                                    "obstacles": obstacles,
                                    "agents": agents,
                                    "timestep": start,
                                    "interval": interval,
                                    "rate": rate,
                                    "value": value,
                                    "time": time,
                                    "feasibility": feasibility == "n" and "exhaustive" or "heuristic",
                                    "cycle": cycle == "n" and "semi-naive" or (
                                            cycle == "o" and "naive" or "proposed"),
                                    "execution_time": execution_time,
                                    "first_agent_arriving": first_agent_arriving,
                                    "cycle_count": cycle_count,
                                    "cycle_agents": cycle_agents,
                                    "unblocked_agents": unblocked_agents,
                                    "feasibility_count": feasibility_count,
                                    "feasibility_type_a": feasibility_type_a,
                                    "feasibility_type_b": feasibility_type_b,
                                    "feasibility_type_c": feasibility_type_c,
                                }
                                main_df = main_df.append(row, ignore_index=True)
    return main_df


def main():
    df_infinite = parse_data("infinite")
    df_periodic = parse_data("periodic")
    df_infinite.to_csv(os.path.join(data_dir, "df_infinite.csv"))
    df_periodic.to_csv(os.path.join(data_dir, "df_periodic.csv"))


if __name__ == '__main__':
    main()
