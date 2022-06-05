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
simulators_list = ["online"]


def parse_data(data_type) -> pandas.DataFrame:
    if data_type == "infinite":
        timesteps_list = [1, 5, 10]
        intervals_list = [0]
    elif data_type == "periodic":
        timesteps_list = [0]
        intervals_list = [1, 5, 10]
    else:
        assert False

    main_df = pandas.DataFrame()
    for obstacles in obstacles_list:
        for simulator in simulators_list:
            for agents in agents_list:
                for timestep in timesteps_list:
                    for interval in intervals_list:
                        for rate in [0.2, 0.4]:
                            for feasibility, cycle in [("h", "h"), ("h", "n"), ("n", "h"), ("n", "n"), ("n", "o"),
                                                       ("h", "o")]:
                                # for feasibility, cycle in [("h", "h"), ("n", "h")]:
                                file = f"{simulator}-{obstacles}-{agents}-{timestep}-{rate}-{interval}-{feasibility}-{cycle}.csv"
                                try:
                                    df = pandas.read_csv(os.path.join(result_dir, file), header=None)
                                    value = npy.around(npy.mean(df.iloc[:, 0]), 3)
                                    time = npy.mean(df.iloc[:, 2])
                                    feasibility_count = 0
                                    cycle_avg = 0
                                    unblocked_avg = 0
                                    feasibility_type_a = 0
                                    feasibility_type_b = 0
                                    feasibility_type_c = 0
                                    if len(df.columns) > 5:
                                        feasibility_count = npy.mean(df.iloc[:, 3])
                                        cycle_avg = npy.mean(df.iloc[:, 4])
                                        unblocked_avg = npy.mean(df.iloc[:, 5])
                                    if len(df.columns) > 9:
                                        feasibility_type_a = npy.mean(df.iloc[:, 6] / df.iloc[:, 3])
                                        feasibility_type_b = npy.mean(df.iloc[:, 9] / df.iloc[:, 3])
                                        feasibility_type_c = npy.mean((df.iloc[:, 7] + df.iloc[:, 8]) / df.iloc[:, 3])
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
                                    "timestep": timestep,
                                    "interval": interval,
                                    "rate": rate,
                                    "value": value,
                                    "time": time,
                                    "feasibility": feasibility == "n" and "exhaustive" or "heuristic",
                                    "cycle": cycle == "o" and "naive (only cycle)" or (
                                            cycle == "n" and "naive" or "proposed"),
                                    "feasibility_count": feasibility_count,
                                    "cycle_avg": cycle_avg,
                                    "unblocked_avg": unblocked_avg,
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
