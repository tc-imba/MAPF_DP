from copy import deepcopy

import click
import dataclasses
import itertools
import numpy as npy
import scipy.stats
import pandas as pd
import os

from motor.motor_asyncio import AsyncIOMotorClient
from pymongo import IndexModel, ASCENDING
from tqdm import tqdm
from loguru import logger

from typing import Optional, Dict, Any, Tuple, List
from pathlib import Path
from experiment.app import app_command, AppArguments
from experiment.utils import project_root, ExperimentSetup, validate_list, asyncio_wrapper

mongo_client = AsyncIOMotorClient()
db = mongo_client["MAPF_DP"]
results_collection = db["results"]
parsed_collection = db["parsed"]


@dataclasses.dataclass
class ParseArguments(AppArguments):
    output_suffix: str
    input_suffix: str
    category: str
    map_names: List[str]
    result_dir: Path
    output_csv: Path
    time_output_csv: Path
    experiment_name: str


# parsed_result_dir = os.path.join(result_dir, "parsed")


header_names_base = [
    'map', 'agent', 'iteration',
    'makespan', 'soc', 'total_time', 'execution_time', 'first_agent_arriving',
]
header_names_online = [
    'cycle_count', 'cycle_agents', 'unblocked_agents', 'feasibility_count',
    'feasibility_1', 'feasibility_2', 'feasibility_3', 'feasibility_4',
    'feasibility_unsettled', 'feasibility_loop', 'feasibility_topo', 'feasibility_recursion',
    'all_node_pairs', 'fixed_node_pairs', 'added_node_pairs'
]
header_names_replan = [
    'partial_replan_count', 'partial_replan_time', 'full_replan_count', 'full_replan_time',
]
column_names = [
    "timing", "simulator", "obstacles", "agents", "k_neighbor", "map_name",
    "delay_start", "delay_interval", "delay_ratio", "delay_type",
    "makespan", "soc", "total_time", "makespan_time", "feasibility", "cycle",
    "execution_time", "first_agent_arriving",
    "cycle_count", "cycle_agents", "unblocked_agents", "feasibility_count",
    "feasibility_type_a", "feasibility_type_b", "feasibility_type_c",
    "average_timestep_time", "average_feasibility_time", "average_cycle_time",
    'average_feasibility_unsettled', 'average_feasibility_loop',
    'average_feasibility_topo', 'average_feasibility_recursion',
    "makespan_lower", "makespan_upper", "soc_lower", "soc_upper",
    "average_timestep_time_lower", "average_timestep_time_upper",
    "makespan_time_lower", "makespan_time_upper",
    'partial_replan_count', 'partial_replan_time', 'full_replan_count', 'full_replan_time',
    "data_points", 'all_node_pairs', 'fixed_node_pairs', 'added_node_pairs'
]
time_header_names = [
    'map', 'agent', 'iteration', 'length', 'data',
]
time_column_names = [
    "timing", "simulator", "obstacles", "agents", "k_neighbor", "map_name",
    "delay_start", "delay_interval", "delay_ratio", "delay_type",
    "data",
]
# feasibility_cycle_enums = [("h", "h"), ("h", "n"), ("n", "h"), ("n", "n"), ("n", "o"), ("h", "o")]
feasibility_cycle_enums = [("h", "h"), ("h", "n"), ("n", "h"), ("n", "n"), ("n", "o"), ("h", "o")]


async def init_db():
    index1 = IndexModel([("setup", ASCENDING)], name="setup")
    index2 = IndexModel([("seeds", ASCENDING)], name="seeds")
    await results_collection.create_indexes([index1, index2])


def get_confidence_interval(data):
    std = scipy.stats.sem(data)
    if std == 0:
        return 0, 0
    return scipy.stats.t.interval(0.95, len(data) - 1, loc=npy.mean(data), scale=std)


class ParsedResult:
    def __init__(self, df: pd.DataFrame):
        self.result = {}
        self.df = df

    def dict(self):
        return self.result

    def add_mean(self, column: str, ci=False):
        if column not in self.df.columns:
            self.result[column] = None
        else:
            self.result[column] = self.df[column].mean()
        if ci:
            if self.result[column] is None:
                lower, upper = None, None
            else:
                lower, upper = get_confidence_interval(self.df[column])
            self.result[f"{column}_lower"] = lower
            self.result[f"{column}_upper"] = upper
            # logger.info("{} {} {} {} {}", column, self.result[column], lower, upper, self.df[column].to_list())

    def add_div_mean(self, column_p: str, column_q: str, column_target: str, ci=False):
        if column_p not in self.df.columns or column_q not in self.df.columns:
            data = None
            self.result[column_target] = None
        else:
            data = npy.ma.masked_invalid(self.df[column_p] / self.df[column_q])
            self.result[column_target] = data.mean() or None
        if ci:
            if data is None:
                lower, upper = None, None
            else:
                lower, upper = get_confidence_interval(data)
            self.result[f"{column_target}_lower"] = lower
            self.result[f"{column_target}_upper"] = upper


async def parse_setup(args: ParseArguments, setup: ExperimentSetup):
    setup_dict = setup.get_filter_dict()
    # logger.info(setup_dict)
    filter_dict = {
        **setup_dict,
        "success": True,
    }
    cursor = results_collection.find(filter_dict)
    results = []
    async for document in cursor:
        if "result" in document:
            result = {
                **document["result"],
                **document["seeds"],
                "simulator": document["setup"]["simulator"],
                "details": document.get("details", [])
            }
            results.append(result)

    if len(results) == 0:
        return None

    df = pd.DataFrame(results)
    # logger.info("{} {}", len(df), setup)
    return df


def parse_merged_df(setup: ExperimentSetup, df: pd.DataFrame) -> Optional[Dict[str, Any]]:
    try:
        result = ParsedResult(df)
        result.add_mean("makespan", ci=True)
        result.add_mean("average_cost", ci=True)
        result.add_mean("total_time")
        result.add_div_mean("total_time", "makespan", "makespan_time", ci=True)

        # if setup.simulator.startswith(("replan", "prioritized")):
        #     result.add_div_mean("total_time", "full_replan_count", "replan_makespan_time", ci=True)
        # else:
        #     result.add_div_mean("total_time", "makespan", "replan_makespan_time", ci=True)
        return result.dict()
    except:
        return None

    try:
        data_points = len(df)
        makespan = npy.mean(df['makespan'])
        makespan_lower, makespan_upper = get_confidence_interval(df['makespan'])
        soc = npy.mean(df['soc'])
        soc_lower, soc_upper = get_confidence_interval(df['soc'])
        # dirty fix, remove in the future
        # if (setup.simulator == "default" or setup.simulator == "online") and setup.delay_interval > 0 and (setup.obstacles != 180 or not (setup.obstacles == 270 and setup.delay_interval == 20 and setup.cycle == "o")):
        #     soc += 1
        #     soc_lower += 1
        #     soc_upper += 1
        total_time = npy.mean(df['total_time'])
        if setup.simulator.startswith(("replan", "prioritized")):
            makespan_time = npy.mean(df['total_time'] / df['full_replan_count'])
            makespan_time_lower, makespan_time_upper = get_confidence_interval(
                df['total_time'] / df['full_replan_count'])
        else:
            makespan_time = npy.mean(df['total_time'] / df['makespan'])
            makespan_time_lower, makespan_time_upper = get_confidence_interval(df['total_time'] / df['makespan'])
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
        average_timestep_time = timestep_time.mean() or 0
        average_timestep_time_lower, average_timestep_time_upper = \
            get_confidence_interval(timestep_time)
        all_node_pairs = 0
        fixed_node_pairs = 0
        added_node_pairs = 0

        if "online" in setup.simulator:
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

            if len(df.columns) > 20:
                all_node_pairs = npy.mean(df['all_node_pairs'])
                fixed_node_pairs = npy.mean(df['fixed_node_pairs'])
                added_node_pairs = npy.mean(df['added_node_pairs'])

        elif setup.simulator.startswith(("replan", "prioritized")):
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
        "k_neighbor": setup.k_neighbor,
        "map_name": setup.map_name,
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
        "makespan_time": makespan_time,
        "makespan_time_lower": makespan_time_lower,
        "makespan_time_upper": makespan_time_upper,
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
        'data_points': data_points,
        'all_node_pairs': all_node_pairs,
        'fixed_node_pairs': fixed_node_pairs,
        'added_node_pairs': added_node_pairs,
    }
    return row


def parse_merged_time_df(setup: ExperimentSetup, df: pd.DataFrame) -> Optional[Dict[str, Any]]:
    num_cases = len(df)
    if setup.simulator == "btpg":
        df["length"] = df.apply(lambda row: len(row["details"][1:]), axis=1)
        df["time"] = df.apply(lambda row: list(map(lambda x: x["execution_time"], row["details"][1:])), axis=1)
    else:
        df["length"] = df.apply(lambda row: len(row["details"]), axis=1)
        df["time"] = df.apply(lambda row: list(map(lambda x: x["execution_time"], row["details"])), axis=1)
    # logger.info(df["time"])
    df = df.explode("time").dropna()
    df.sort_values(by="time", inplace=True)

    # df["time"] = df["time"].apply(lambda x: npy.ceil(x * 1000) / 1000)
    df["prob"] = 1 / df["length"] / num_cases
    df["cdf"] = df["prob"].cumsum()
    # df = df.drop_duplicates(subset=['time'], keep='last')

    df["new_data"] = df[["time", "prob", "cdf"]].apply(lambda x: tuple(x), axis=1)
    # logger.info(df["new_data"].to_list())
    return df["new_data"].to_list()

    # row = {
    #     "timing": setup.timing,
    #     "simulator": setup.simulator,
    #     "obstacles": setup.obstacles,
    #     "agents": setup.agents,
    #     "k_neighbor": setup.k_neighbor,
    #     "map_name": setup.map_name,
    #     "delay_start": setup.delay_start,
    #     "delay_interval": setup.delay_interval,
    #     "delay_ratio": setup.delay_ratio,
    #     "delay_type": setup.delay_type,
    #     "data": list(df["new_data"])
    # }
    # return row


async def parse_data(args: ParseArguments) -> Tuple[pd.DataFrame, pd.DataFrame]:
    # if data_type == "infinite":
    #     starts_list = [1, 5, 10]
    #     intervals_list = [0]
    # elif data_type == "periodic":
    #     starts_list = [1]
    #     intervals_list = [1, 10, 20, 30]
    # else:
    #     assert False

    rows = []
    time_rows = []
    cases = list(itertools.product(
        args.delay_types,
        args.obstacles,
        args.agents,
        args.k_neighbors,
        args.map_names,
        args.delay_intervals,
        args.delay_ratios
    ))
    for case in tqdm(cases):
        logger.info("Parse {}", case)
        delay_type, obstacles, agents, k_neighbor, map_name, delay_interval, delay_ratio = case
        simulator_feasibility_cycle = []
        for simulator in args.simulators:
            if simulator == "online":
                for feasibility, cycle in feasibility_cycle_enums:
                    simulator_feasibility_cycle.append((simulator, feasibility, cycle))
            else:
                simulator_feasibility_cycle.append((simulator, "h", "h"))

        raw_dfs = {}
        time_dfs = {}
        setups = {}

        if args.timing == "discrete":
            solver = "eecbs"
        else:
            solver = "ccbs"

        for simulator, feasibility, cycle in simulator_feasibility_cycle:
            # for simulator in args.simulators:
            # if simulator == "online":
            #     _feasibility = feasibility
            #     _cycle = cycle
            # else:
            #     _feasibility = "h"
            #     _cycle = "h"
            label = f"{simulator}-{feasibility}-{cycle}"
            setup = ExperimentSetup(
                timing=args.timing, map=args.map, map_name=map_name,
                solver=solver, simulator=simulator, obstacles=obstacles,
                k_neighbor=k_neighbor, agents=agents, delay_type=delay_type,
                delay_ratio=delay_ratio, delay_start=0, delay_interval=delay_interval,
                feasibility=feasibility, cycle=cycle,
            )
            setups[label] = setup
            df = await parse_setup(args, setup)
            if df is not None:
                raw_dfs[label] = df

        # if delay_interval == 20 and obstacles == 270:
        #     print(raw_dfs)

        # if len(raw_dfs) != len(args.simulators):
        #     continue

        if "online_opt-h-h" in raw_dfs.keys() and "online-h-h" not in raw_dfs.keys():
            raw_dfs["online-h-h"] = raw_dfs["online_opt-h-h"]
            setups["online-h-h"] = deepcopy(setups["online_opt-h-h"])
            setups["online-h-h"].simulator = "online"
        parsed_labels = list(raw_dfs.keys())
        # logger.debug(parsed_labels)
        # print(parsed_labels)
        if len(parsed_labels) == 0:
            continue

        base_df = None
        for label in parsed_labels:
            target_df = raw_dfs[label]
            if base_df is None:
                base_df = target_df
            else:
                base_df = base_df[['map_seed', 'agent_seed', 'simulation_seed']]
                base_df = base_df.merge(target_df, on=['map_seed', 'agent_seed', 'simulation_seed'], how='inner')
        if base_df is None:
            logger.error("error: base_df is None")
            continue
        base_df = base_df[['map_seed', 'agent_seed', 'simulation_seed']]
        # base_df = None
        # base_naive_df = None
        # for label in parsed_labels:
        #     target_df = raw_dfs[label]
        #     # logger.info(target_df.columns)
        #     if label.startswith("online-h"):
        #         if base_naive_df is None:
        #             base_naive_df = target_df
        #         else:
        #             base_naive_df = base_naive_df[['map_seed', 'agent_seed', 'simulation_seed']]
        #             base_naive_df = base_naive_df.merge(target_df, on=['map_seed', 'agent_seed', 'simulation_seed'],
        #                                                 how='inner')
        #     else:
        #         if base_df is None:
        #             base_df = target_df
        #         else:
        #             base_df = base_df[['map_seed', 'agent_seed', 'simulation_seed']]
        #             base_df = base_df.merge(target_df, on=['map_seed', 'agent_seed', 'simulation_seed'], how='inner')
        #
        # if base_df is None or base_naive_df is None:
        #     logger.error("base_df or base_naive_df is None")
        #     continue
        #
        # base_df = base_df[['map_seed', 'agent_seed', 'simulation_seed']]
        # base_naive_df = base_naive_df[['map_seed', 'agent_seed', 'simulation_seed']]

        for label in parsed_labels:
            setup = setups[label]
            df = raw_dfs[label]
            # if label.startswith("online-h"):
            #     df = df.merge(base_naive_df, on=['map_seed', 'agent_seed', 'simulation_seed'], how='inner')
            # else:
            #     df = df.merge(base_df, on=['map_seed', 'agent_seed', 'simulation_seed'], how='inner')
            df = df.merge(base_df, on=['map_seed', 'agent_seed', 'simulation_seed'], how='inner')
            row = parse_merged_df(setup, df)
            if row is not None:
                # rows.append(row)
                cdf_data = parse_merged_time_df(setup, df)
                doc_filter = {
                    "experiment": args.experiment_name,
                    "case": setup.get_output_prefix(),
                }
                doc = {
                    "experiment": args.experiment_name,
                    "case": setup.get_output_prefix(),
                    "setup": setup.dict(),
                    "count": len(df),
                    "result": row,
                    "cdf_data": cdf_data,
                }

                await parsed_collection.replace_one(doc_filter, doc, upsert=True)
                logger.info("{} cases: {}", len(df), setup.get_output_prefix())

                # main_df = pd.DataFrame(data=rows, columns=column_names)
    # return main_df


@app_command("parse")
@click.option('-o', '--output-suffix', default='')
@click.option('-i', '--input-suffix', default='')
@click.option('--category', is_flag=True, default=False)
@click.option("--map-names", type=str, default="random", callback=validate_list(str))
@click.option("--experiment-name", type=str, default="test1")
@click.pass_context
@asyncio_wrapper
async def main(ctx, output_suffix, input_suffix, category, map_names, experiment_name):
    if input_suffix:
        input_suffix = '_' + input_suffix
    if output_suffix:
        output_suffix = '_' + output_suffix

    data_dir = project_root / "data"
    os.makedirs(data_dir, exist_ok=True)
    timing = ctx.obj.timing

    args = ParseArguments(
        **ctx.obj.__dict__,
        output_suffix=output_suffix,
        input_suffix=input_suffix,
        category=category,
        map_names=map_names,
        result_dir=project_root / f"result{input_suffix}",
        output_csv=data_dir / f"df_{timing}{output_suffix}.csv",
        time_output_csv=data_dir / f"df_{timing}_time{output_suffix}.csv",
        experiment_name=experiment_name,
    )
    logger.info(args)
    await init_db()
    await parse_data(args)

    # main_df, time_df = await parse_data(args)
    # main_df.to_csv(args.output_csv, index=False)
    # logger.info("write main_df to {}", args.output_csv)
    # time_df.to_csv(args.time_output_csv, index=False)
    # logger.info("write time_df to {}", args.time_output_csv)

    # df_infinite = parse_data(result_dir, "infinite", category)
    # df_periodic = parse_data(result_dir, "periodic", category)
    #
    # df_infinite.to_csv(os.path.join(data_dir, f"df_infinite{output_suffix}.csv"), index=False)
    # df_periodic.to_csv(os.path.join(data_dir, f"df_periodic{output_suffix}.csv"), index=False)

    # click.echo(args)


if __name__ == '__main__':
    main()
