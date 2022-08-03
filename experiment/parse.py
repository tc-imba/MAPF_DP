import click
import numpy as npy
import pandas
import os

project_root = os.path.dirname(os.path.dirname(__file__))
# parsed_result_dir = os.path.join(result_dir, "parsed")
plot_dir = os.path.join(project_root, "plot")
data_dir = os.path.join(project_root, "data")
os.makedirs(plot_dir, exist_ok=True)
os.makedirs(data_dir, exist_ok=True)
# os.makedirs(parsed_result_dir, exist_ok=True)

# obstacles_list = [90, 180, 270, 360, 450]
obstacles_list = [90, 180, 270]
agents_list = [10, 20, 30]
simulators_list = ["online", "default"]
delay_ratios_list = [0.01, 0.05, 0.1, 0.2, 0.3]
delay_types_list = ["agent", "edge"]


def parse_data(result_dir, data_type) -> pandas.DataFrame:
    if data_type == "infinite":
        starts_list = [1, 5, 10]
        intervals_list = [0]
    elif data_type == "periodic":
        starts_list = [1]
        intervals_list = [10, 20, 30]
    else:
        assert False

    header_names = [
        'map', 'agent', 'iteration',
        'value', 'noop', 'time', 'execution_time', 'first_agent_arriving',
        'cycle_count', 'cycle_agents', 'unblocked_agents', 'feasibility_count',
        'feasibility_1', 'feasibility_2', 'feasibility_3', 'feasibility_4',
    ]
    column_names = [
        "simulator", "agents", "timestep", "interval", "rate", "delay_type",
        "value", "time", "feasibility", "cycle", "execution_time", "first_agent_arriving",
        "cycle_count", "cycle_agents", "unblocked_agents", "feasibility_count",
        "feasibility_type_a", "feasibility_type_b", "feasibility_type_c",
    ]

    main_df = pandas.DataFrame(columns=column_names)
    for delay_type in delay_types_list:
        for obstacles in obstacles_list:
            for agents in agents_list:
                for start in starts_list:
                    for interval in intervals_list:
                        for rate in delay_ratios_list:
                            for feasibility, cycle in [("h", "h"), ("h", "n"), ("n", "h"), ("n", "n"), ("n", "o"),
                                                       ("h", "o")]:
                                raw_dfs = {}

                                for simulator in simulators_list:
                                    # for feasibility, cycle in [("h", "h"), ("n", "h")]:
                                    file = f"{simulator}-{obstacles}-{agents}-{delay_type}-{rate}-{start}-{interval}-{feasibility}-{cycle}.csv"
                                    try:
                                        df = pandas.read_csv(os.path.join(result_dir, file), header=None,
                                                             names=header_names)
                                        df.sort_values(by=['map', 'agent', 'iteration'], inplace=True)
                                        df.to_csv(os.path.join(result_dir, "parsed", file), index=False)
                                        raw_dfs[simulator] = df
                                    except:
                                        pass

                                if len(raw_dfs) != len(simulators_list):
                                    continue

                                base_df = raw_dfs[simulators_list[0]]
                                for simulator in simulators_list[1:]:
                                    target_df = raw_dfs[simulator]
                                    base_df = base_df.merge(target_df, on=['map', 'agent', 'iteration'], how='inner')
                                base_df = base_df[['map', 'agent', 'iteration']]

                                for simulator in simulators_list:
                                    df = raw_dfs[simulator]
                                    df = df.merge(base_df, on=['map', 'agent', 'iteration'], how='inner')

                                    try:
                                        value = npy.mean(df['value'])
                                        time = npy.mean(df['time'])
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
                                            execution_time = npy.mean(df['execution_time'])
                                            first_agent_arriving = npy.mean(df['first_agent_arriving'])
                                            cycle_count = npy.mean(df['cycle_count'])
                                            cycle_agents = npy.mean(df['cycle_agents'])
                                            unblocked_agents = npy.mean(df['unblocked_agents'])
                                            feasibility_count = npy.mean(df['feasibility_count'])
                                        if len(df.columns) > 12:
                                            feasibility_count_all = npy.sum(
                                                df[['feasibility_1', 'feasibility_2', 'feasibility_3',
                                                    'feasibility_4']],
                                                axis=1
                                            )
                                            feasibility_type_a = npy.nanmean(
                                                df['feasibility_1'] / feasibility_count_all)
                                            feasibility_type_b = npy.nanmean(
                                                df['feasibility_4'] / feasibility_count_all)
                                            feasibility_type_c = npy.nanmean(
                                                (df['feasibility_2'] + df['feasibility_3']) / feasibility_count_all)
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
                                        "delay_type": delay_type,
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


@click.command()
@click.option('-o', '--output-suffix', default='')
@click.option('-i', '--input-suffix', default='')
def main(output_suffix, input_suffix):
    if input_suffix:
        input_suffix = '_' + input_suffix
    if output_suffix:
        output_suffix = '_' + output_suffix

    result_dir = os.path.join(project_root, f"result{input_suffix}")

    df_infinite = parse_data(result_dir, "infinite")
    df_periodic = parse_data(result_dir, "periodic")

    df_infinite.to_csv(os.path.join(data_dir, f"df_infinite{output_suffix}.csv"), index=False)
    df_periodic.to_csv(os.path.join(data_dir, f"df_periodic{output_suffix}.csv"), index=False)


if __name__ == '__main__':
    main()
