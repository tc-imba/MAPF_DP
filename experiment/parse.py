import click
import dataclasses
import itertools
import numpy as npy
import scipy.stats
import pandas as pd
import os
from tqdm import tqdm

from typing import Optional, Dict, Any
from pathlib import Path
from experiment.app import app_command, AppArguments
from experiment.utils import project_root, ExperimentSetup


@dataclasses.dataclass
class ParseArguments(AppArguments):
    output_suffix: str
    input_suffix: str
    category: str
    timing: str
    result_dir: Path
    output_csv: Path


# parsed_result_dir = os.path.join(result_dir, "parsed")


header_names_base = [
    'map', 'agent', 'iteration',
    'makespan', 'soc', 'total_time', 'execution_time', 'first_agent_arriving',
]
header_names_online = [
    'cycle_count', 'cycle_agents', 'unblocked_agents', 'feasibility_count',
    'feasibility_1', 'feasibility_2', 'feasibility_3', 'feasibility_4',
    'feasibility_unsettled', 'feasibility_loop', 'feasibility_topo', 'feasibility_recursion',
]
header_names_replan = [
    'partial_replan_count', 'partial_replan_time', 'full_replan_count', 'full_replan_time',
]
column_names = [
    "timing", "simulator", "obstacles", "agents",
    "delay_start", "delay_interval", "delay_ratio", "delay_type",
    "makespan", "soc", "total_time", "feasibility", "cycle",
    "execution_time", "first_agent_arriving",
    "cycle_count", "cycle_agents", "unblocked_agents", "feasibility_count",
    "feasibility_type_a", "feasibility_type_b", "feasibility_type_c",
    "average_timestep_time", "average_feasibility_time", "average_cycle_time",
    'average_feasibility_unsettled', 'average_feasibility_loop',
    'average_feasibility_topo', 'average_feasibility_recursion',
    "makespan_lower", "makespan_upper", "soc_lower", "soc_upper",
    "average_timestep_time_lower", "average_timestep_time_upper",
    'partial_replan_count', 'partial_replan_time', 'full_replan_count', 'full_replan_time',
]
feasibility_cycle_enums = [("h", "h"), ("h", "n"), ("n", "h"), ("n", "n"), ("n", "o"), ("h", "o")]


def get_confidence_interval(data):
    std = scipy.stats.sem(data)
    if std == 0:
        return 0, 0
    return scipy.stats.t.interval(0.95, len(data) - 1, loc=npy.mean(data), scale=std)


def parse_raw_csv(args: ParseArguments, setup: ExperimentSetup) -> Optional[pd.DataFrame]:
    output_prefix = setup.get_output_prefix()
    output_file = f"{output_prefix}.csv"
    # click.echo(output_file)
    try:
        if setup.simulator == "online":
            header_names = header_names_base + header_names_online
        elif setup.simulator in ("replan", "prioritized", "prioritized_opt"):
            header_names = header_names_base + header_names_replan
        else:
            header_names = header_names_base
        df = pd.read_csv(args.result_dir / output_file, header=None,
                         names=header_names)
        df.sort_values(by=['map', 'agent', 'iteration'], inplace=True)
        # df.to_csv(args.result_dir / "parsed" / output_file, index=False)
        if len(df) > 0 and setup.simulator == "online":
            condition = (df['feasibility_1'] == 0) & (df['feasibility_2'] == 0) & (
                    df['feasibility_3'] == 0) & (df['feasibility_4'] == 0)
            if args.category:
                df = df[~condition]
            else:
                df = df[condition]
        return df
    except Exception as e:
        return None


def parse_merged_df(setup: ExperimentSetup, df: pd.DataFrame) -> Optional[Dict[str, Any]]:
    try:
        makespan = npy.mean(df['makespan'])
        makespan_lower, makespan_upper = get_confidence_interval(df['makespan'])
        soc = npy.mean(df['soc'])
        soc_lower, soc_upper = get_confidence_interval(df['soc'])
        # dirty fix, remove in the future
        if setup.simulator == "default" or setup.simulator == "online":
            soc += 1
            soc_lower += 1
            soc_upper += 1
        total_time = npy.mean(df['total_time'])
        execution_time = 0
        first_agent_arriving = 0
        cycle_count = 0
        cycle_agents = 0
        unblocked_agents = 0
        feasibility_count = 0
        average_timestep_time = 0
        average_timestep_time_lower = 0
        average_timestep_time_upper = 0
        average_feasibility_time = 0
        average_cycle_time = 0
        feasibility_type_a = 0
        feasibility_type_b = 0
        feasibility_type_c = 0
        average_feasibility_unsettled = 0
        average_feasibility_loop = 0
        average_feasibility_topo = 0
        average_feasibility_recursion = 0
        partial_replan_count = 0
        partial_replan_time = 0
        full_replan_count = 0
        full_replan_time = 0
        execution_time = npy.mean(df['execution_time'])
        first_agent_arriving = npy.mean(df['first_agent_arriving'])
        timestep_time = npy.ma.masked_invalid(
            df['execution_time'] / df['first_agent_arriving'])
        average_timestep_time = timestep_time.mean() or None
        average_timestep_time_lower, average_timestep_time_upper = \
            get_confidence_interval(timestep_time)

        if setup.simulator == "online":
            cycle_count = npy.mean(df['cycle_count'])
            cycle_agents = npy.mean(df['cycle_agents'])
            unblocked_agents = npy.mean(df['unblocked_agents'])
            feasibility_count = npy.mean(df['feasibility_count'])
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
                average_feasibility_unsettled = npy.mean(
                    df['feasibility_unsettled'] / df['first_agent_arriving'])
                average_feasibility_loop = npy.mean(
                    df['feasibility_loop'] / df['first_agent_arriving'])
                average_feasibility_topo = npy.mean(
                    df['feasibility_topo'] / df['first_agent_arriving'])
                average_feasibility_recursion = npy.mean(
                    df['feasibility_recursion'] / df['first_agent_arriving'])
        elif setup.simulator in ("replan", "prioritized", "prioritized_opt"):
            partial_replan_count = npy.mean(df['partial_replan_count'])
            partial_replan_time = npy.mean(df['partial_replan_time'])
            full_replan_count = npy.mean(df['full_replan_count'])
            full_replan_time = npy.mean(df['full_replan_time'])
    except:
        total_time = 0
    if total_time == 0 or npy.isnan(total_time):
        return None
    row = {
        "timing": setup.timing,
        "simulator": setup.simulator,
        "obstacles": setup.obstacles,
        "agents": setup.agents,
        "delay_start": setup.delay_start,
        "delay_interval": setup.delay_interval,
        "delay_ratio": setup.delay_ratio,
        "delay_type": setup.delay_type,
        "makespan": makespan,
        "makespan_lower": makespan_lower,
        "makespan_upper": makespan_upper,
        "soc": soc,
        "soc_lower": soc_lower,
        "soc_upper": soc_upper,
        "total_time": total_time,
        "feasibility": setup.feasibility == "n" and "exhaustive" or "heuristic",
        "cycle": setup.cycle == "n" and "semi-naive" or (
                setup.cycle == "o" and "naive" or "proposed"),
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
        "average_timestep_time_lower": average_timestep_time_lower,
        "average_timestep_time_upper": average_timestep_time_upper,
        'partial_replan_count': partial_replan_count,
        'partial_replan_time': partial_replan_time,
        'full_replan_count': full_replan_count,
        'full_replan_time': full_replan_time,
    }
    return row


def parse_data(args: ParseArguments) -> pd.DataFrame:
    # if data_type == "infinite":
    #     starts_list = [1, 5, 10]
    #     intervals_list = [0]
    # elif data_type == "periodic":
    #     starts_list = [1]
    #     intervals_list = [1, 10, 20, 30]
    # else:
    #     assert False

    rows = []
    cases = list(itertools.product(
        args.delay_types,
        args.obstacles,
        args.agents,
        args.k_neighbors,
        args.delay_intervals,
        args.delay_ratios
    ))
    for case in tqdm(cases):
        delay_type, obstacles, agents, k_neighbor, delay_interval, delay_ratio = case
        for feasibility, cycle in feasibility_cycle_enums:
            raw_dfs = {}
            setups = {}

            for simulator in args.simulators:
                if simulator == "online":
                    _feasibility = feasibility
                    _cycle = cycle
                else:
                    _feasibility = "h"
                    _cycle = "h"

                setup = ExperimentSetup(
                    timing=args.timing, simulator=simulator, obstacles=obstacles,
                    k_neighbor=k_neighbor, agents=agents, delay_type=delay_type,
                    delay_ratio=delay_ratio, delay_start=0, delay_interval=delay_interval,
                    feasibility=_feasibility, cycle=_cycle,
                )
                setups[simulator] = setup
                df = parse_raw_csv(args, setup)
                if df is not None:
                    raw_dfs[simulator] = df

            if len(raw_dfs) != len(args.simulators):
                continue

            base_df = raw_dfs[args.simulators[0]]
            for simulator in args.simulators[1:]:
                target_df = raw_dfs[simulator]
                base_df = base_df[['map', 'agent', 'iteration']]
                base_df = base_df.merge(target_df, on=['map', 'agent', 'iteration'], how='inner')
            base_df = base_df[['map', 'agent', 'iteration']]

            for simulator in args.simulators:
                if (simulator != "online") and (feasibility != "h" or cycle != "h"):
                    continue
                if simulator not in raw_dfs:
                    continue
                df = raw_dfs[simulator]
                df = df.merge(base_df, on=['map', 'agent', 'iteration'], how='inner')
                row = parse_merged_df(setups[simulator], df)
                if row is not None:
                    rows.append(row)

    main_df = pd.DataFrame(data=rows, columns=column_names)
    return main_df


@app_command("parse")
@click.option('-o', '--output-suffix', default='')
@click.option('-i', '--input-suffix', default='')
@click.option('--category', is_flag=True, default=False)
@click.option('--timing', default='discrete')
@click.pass_context
def main(ctx, output_suffix, input_suffix, category, timing):
    if input_suffix:
        input_suffix = '_' + input_suffix
    if output_suffix:
        output_suffix = '_' + output_suffix

    data_dir = project_root / "data"
    os.makedirs(data_dir, exist_ok=True)

    args = ParseArguments(
        **ctx.obj.__dict__,
        output_suffix=output_suffix,
        input_suffix=input_suffix,
        category=category,
        timing=timing,
        result_dir=project_root / f"result{input_suffix}",
        output_csv=data_dir / f"df_{timing}{output_suffix}.csv",
    )
    click.echo(args)

    df = parse_data(args)
    df.to_csv(args.output_csv, index=False)

    # df_infinite = parse_data(result_dir, "infinite", category)
    # df_periodic = parse_data(result_dir, "periodic", category)
    #
    # df_infinite.to_csv(os.path.join(data_dir, f"df_infinite{output_suffix}.csv"), index=False)
    # df_periodic.to_csv(os.path.join(data_dir, f"df_periodic{output_suffix}.csv"), index=False)

    # click.echo(args)


if __name__ == '__main__':
    main()
