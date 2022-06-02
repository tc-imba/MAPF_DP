import numpy as npy
import pandas
import os

project_root = os.path.dirname(os.path.dirname(__file__))
result_dir = os.path.join(project_root, "result")
plot_dir = os.path.join(project_root, "plot")
os.makedirs(plot_dir, exist_ok=True)

# obstacles_list = [90, 180, 270, 360, 450]
obstacles_list = [90, 180, 270]
agents_list = [10]
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
                                    scores = df.iloc[:, 0]
                                    value = npy.around(npy.mean(scores), 3)
                                    seconds = df.iloc[:, 2]
                                    time = npy.mean(seconds)
                                except:
                                    value = 0
                                    time = 0
                                # print(file, mean)
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
                                    "cycle": cycle == "n" and "naive (only cycle)" or (
                                            cycle == "o" and "naive" or "proposed")
                                }
                                main_df = main_df.append(row, ignore_index=True)
    return main_df


def main():
    df_infinite = parse_data("infinite")
    df_periodic = parse_data("periodic")
    df_infinite.to_csv(os.path.join(project_root, "df_infinite.csv"))
    df_periodic.to_csv(os.path.join(project_root, "df_periodic.csv"))


if __name__ == '__main__':
    main()
