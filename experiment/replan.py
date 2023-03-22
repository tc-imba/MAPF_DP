import click
import numpy as npy
import scipy.stats
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
delay_ratios_list = [0.1, 0.2, 0.3]
delay_types_list = ["agent"]

header_names = [
    'map', 'agent', 'iteration',
    'makespan', 'value', 'time', 'execution_time', 'first_agent_arriving',
    'cycle_count', 'cycle_agents', 'unblocked_agents', 'feasibility_count',
    'feasibility_1', 'feasibility_2', 'feasibility_3', 'feasibility_4',
    'feasibility_unsettled', 'feasibility_loop', 'feasibility_topo', 'feasibility_recursion',
]


def get_confidence_interval(data):
    return scipy.stats.t.interval(0.95, len(data)-1, loc=npy.mean(data), scale=scipy.stats.sem(data))



def parse_data(result_dir, data_type, category) -> pandas.DataFrame:
    print(1)
    if data_type == "infinite":
        starts_list = [1, 5, 10]
        intervals_list = [0]
    elif data_type == "periodic":
        starts_list = [1]
        intervals_list = [1, 10, 20, 30]
    else:
        assert False

    rows = []

    for delay_type in delay_types_list:
        for obstacles in obstacles_list:
            for agents in agents_list:
                for start in starts_list:
                    for interval in intervals_list:
                        for rate in delay_ratios_list:
                            file = f"replan-{obstacles}-{agents}-{delay_type}-{rate}-{start}-{interval}-h-h.csv"
                            try:
                                _df = pandas.read_csv(os.path.join(result_dir, file), header=None,
                                                     names=header_names)
                                _df.sort_values(by=['map', 'agent', 'iteration'], inplace=True)
                            except:
                                continue

                            for (map_seed, agent_seed), df in _df.groupby(['map', 'agent']):
                                value = npy.mean(df['value'])
                                value_lower, value_upper = get_confidence_interval(df['value'])
                                timestep_time = npy.ma.masked_invalid(
                                    df['execution_time'] / df['first_agent_arriving'])
                                average_timestep_time = timestep_time.mean() or None
                                average_timestep_time_lower, average_timestep_time_upper = \
                                    get_confidence_interval(timestep_time)
                                execution_time = npy.mean(df['execution_time'])
                                first_agent_arriving = npy.mean(df['first_agent_arriving'])
                                row = {
                                    "simulator": "replan",
                                    "obstacles": obstacles,
                                    "agents": agents,
                                    "timestep": start,
                                    "interval": interval,
                                    "rate": rate,
                                    "delay_type": delay_type,
                                    "map_seed": map_seed,
                                    "agent_seed": agent_seed,
                                    "value": value,
                                    "value_lower": value_lower,
                                    "value_upper": value_upper,
                                    "execution_time": execution_time,
                                    "first_agent_arriving": first_agent_arriving,
                                    "average_timestep_time": average_timestep_time,
                                    "average_timestep_time_lower": average_timestep_time_lower,
                                    "average_timestep_time_upper": average_timestep_time_upper,

                                }
                                rows.append(row)
    main_df = pandas.DataFrame(data=rows)
    print(main_df)
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

    # df_infinite = parse_data(result_dir, "infinite", category)
    df_periodic = parse_data(result_dir, "periodic", category)

    # df_infinite.to_csv(os.path.join(data_dir, f"df_replan_infinite{output_suffix}.csv"), index=False)
    df_periodic.to_csv(os.path.join(data_dir, f"df_replan_periodic{output_suffix}.csv"), index=False)


if __name__ == '__main__':
    main()
