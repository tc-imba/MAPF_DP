import click
import numpy as npy
import pandas
import os

project_root = os.path.dirname(os.path.dirname(__file__))
plot_dir = os.path.join(project_root, "plot")
data_dir = os.path.join(project_root, "data")
result_dir = os.path.join(project_root, "result")
parsed_result_dir = os.path.join(result_dir, "parsed")
os.makedirs(plot_dir, exist_ok=True)
os.makedirs(data_dir, exist_ok=True)
os.makedirs(parsed_result_dir, exist_ok=True)

# obstacles_list = [90, 180, 270, 360, 450]
obstacles_list = [90, 180, 270]
agents_list = [10, 20, 30]
simulators_list = ["online", "replan", "default"]
delay_ratios_list = [0, 0.01, 0.05, 0.1, 0.2, 0.3]
delay_types_list = ["agent", "edge"]


def parse_data(result_dir, data_type, category) -> pandas.DataFrame:
    if data_type == "infinite":
        starts_list = [1, 5, 10]
        intervals_list = [0]
    elif data_type == "periodic":
        starts_list = [1]
        intervals_list = [1, 10, 20, 30]
    else:
        assert False

    header_names = [
        'map', 'agent', 'iteration',
        'makespan', 'value', 'time', 'execution_time', 'first_agent_arriving',
        'cycle_count', 'cycle_agents', 'unblocked_agents', 'feasibility_count',
        'feasibility_1', 'feasibility_2', 'feasibility_3', 'feasibility_4',
        'feasibility_unsettled', 'feasibility_loop', 'feasibility_topo', 'feasibility_recursion',
    ]
    column_names = [
        "simulator", "agents", "timestep", "interval", "rate", "delay_type",
        "makespan", "value", "time", "feasibility", "cycle", "execution_time", "first_agent_arriving",
        "cycle_count", "cycle_agents", "unblocked_agents", "feasibility_count",
        "feasibility_type_a", "feasibility_type_b", "feasibility_type_c",
        "average_timestep_time", "average_feasibility_time", "average_cycle_time",
        'average_feasibility_unsettled', 'average_feasibility_loop',
        'average_feasibility_topo', 'average_feasibility_recursion',
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
                                    if simulator == "default" or simulator == "replan":
                                        _feasibility = "h"
                                        _cycle = "h"
                                    else:
                                        _feasibility = feasibility
                                        _cycle = cycle
                                    file = f"{simulator}-{obstacles}-{agents}-{delay_type}-{rate}-{start}-{interval}-{_feasibility}-{_cycle}.csv"
                                    try:
                                        df = pandas.read_csv(os.path.join(result_dir, file), header=None,
                                                             names=header_names)
                                        df.sort_values(by=['map', 'agent', 'iteration'], inplace=True)
                                        df.to_csv(os.path.join(result_dir, "parsed", file), index=False)
                                        if len(df) > 0 and simulator == "online":
                                            condition = (df['feasibility_1'] == 0) & (df['feasibility_2'] == 0) & (
                                                        df['feasibility_3'] == 0) & (df['feasibility_4'] == 0)
                                            if category:
                                                df = df[~condition]
                                            else:
                                                df = df[condition]
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
                                    if (simulator == "default" or simulator == "replan") and (feasibility != "h" or cycle != "h"):
                                        continue

                                    df = raw_dfs[simulator]
                                    df = df.merge(base_df, on=['map', 'agent', 'iteration'], how='inner')

                                    try:
                                        value = npy.mean(df['value'])
                                        time = npy.mean(df['time'])
                                        makespan = npy.mean(df['makespan'])
                                        execution_time = 0
                                        first_agent_arriving = 0
                                        cycle_count = 0
                                        cycle_agents = 0
                                        unblocked_agents = 0
                                        feasibility_count = 0
                                        feasibility_type_a = 0
                                        feasibility_type_b = 0
                                        feasibility_type_c = 0
                                        average_feasibility_unsettled = 0
                                        average_feasibility_loop = 0
                                        average_feasibility_topo = 0
                                        average_feasibility_recursion = 0
                                        if len(df.columns) > 8:
                                            execution_time = npy.mean(df['execution_time'])
                                            first_agent_arriving = npy.mean(df['first_agent_arriving'])
                                            cycle_count = npy.mean(df['cycle_count'])
                                            cycle_agents = npy.mean(df['cycle_agents'])
                                            unblocked_agents = npy.mean(df['unblocked_agents'])
                                            feasibility_count = npy.mean(df['feasibility_count'])
                                            average_timestep_time = npy.ma.masked_invalid(
                                                df['execution_time'] / df['first_agent_arriving']).mean() or None
                                            average_feasibility_time = npy.ma.masked_invalid(
                                                df['execution_time'] / df['feasibility_count']).mean() or None
                                            average_cycle_time = npy.ma.masked_invalid(
                                                df['execution_time'] / df['cycle_count']).mean() or None
                                        if len(df.columns) > 12:
                                            feasibility_count_all = npy.sum(
                                                df[['feasibility_1', 'feasibility_2', 'feasibility_3',
                                                    'feasibility_4']],
                                                axis=1
                                            )
                                            if npy.sum(df['feasibility_1'] > 0):
                                                feasibility_type_a = npy.nanmean(
                                                    df['feasibility_1'] / feasibility_count_all)
                                            if npy.sum(df['feasibility_4'] > 0):
                                                feasibility_type_b = npy.nanmean(
                                                    df['feasibility_4'] / feasibility_count_all)
                                            if npy.sum(df['feasibility_2'] > 0) or npy.sum(df['feasibility_3'] > 0):
                                                feasibility_type_c = npy.nanmean(
                                                    (df['feasibility_2'] + df['feasibility_3']) / feasibility_count_all)
                                        if len(df.columns) > 16:
                                            average_feasibility_unsettled = npy.mean(df['feasibility_unsettled'] / df['first_agent_arriving'])
                                            average_feasibility_loop = npy.mean(df['feasibility_loop'] / df['first_agent_arriving'])
                                            average_feasibility_topo = npy.mean(df['feasibility_topo'] / df['first_agent_arriving'])
                                            average_feasibility_recursion = npy.mean(df['feasibility_recursion'] / df['first_agent_arriving'])
                                    except:
                                        value = 0
                                        time = 0
                                    if time == 0 or npy.isnan(time):
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
                                        "makespan": makespan,
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
                                        "average_timestep_time": average_timestep_time,
                                        "average_feasibility_time": average_feasibility_time,
                                        "average_cycle_time": average_cycle_time,
                                        "average_feasibility_unsettled": average_feasibility_unsettled,
                                        "average_feasibility_loop": average_feasibility_loop,
                                        "average_feasibility_topo": average_feasibility_topo,
                                        "average_feasibility_recursion": average_feasibility_recursion,
                                    }
                                    main_df = main_df.append(row, ignore_index=True)

    return main_df


@click.command()
@click.option('-o', '--output-suffix', default='')
@click.option('-i', '--input-suffix', default='')
@click.option('--category', is_flag=True, default=False)
def main(output_suffix, input_suffix, category):
    if input_suffix:
        input_suffix = '_' + input_suffix
    if output_suffix:
        output_suffix = '_' + output_suffix

    result_dir = os.path.join(project_root, f"result{input_suffix}")

    df_infinite = parse_data(result_dir, "infinite", category)
    df_periodic = parse_data(result_dir, "periodic", category)

    df_infinite.to_csv(os.path.join(data_dir, f"df_infinite{output_suffix}.csv"), index=False)
    df_periodic.to_csv(os.path.join(data_dir, f"df_periodic{output_suffix}.csv"), index=False)


if __name__ == '__main__':
    main()
