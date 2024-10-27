import asyncio
import math
import os
import time
from typing import List
import bson

import click
import subprocess
import platform
from pathlib import Path
import multiprocessing
import concurrent.futures
import pandas as pd
import dataclasses
from tqdm import tqdm
import psutil
from motor.motor_asyncio import AsyncIOMotorClient
from pymongo import IndexModel, ASCENDING, DESCENDING
import aiofiles

from experiment.app import app_command, AppArguments
from experiment.utils import asyncio_wrapper, project_root, ExperimentSetup, validate_list
from experiment.logger import logger


@dataclasses.dataclass
class TestArguments(AppArguments):
    experiment_name: str
    map_seeds: int
    map_names: List[str]
    agent_seeds: int
    iteration: int
    timeout: int
    suboptimality: float
    solver: str
    program: Path
    maps_dir: Path
    mapf_maps_dir: Path
    result_dir: Path
    pool: concurrent.futures.ProcessPoolExecutor
    semaphore: asyncio.Semaphore
    naive: bool
    overwrite: bool


# workers = multiprocessing.cpu_count()
# workers = 1
EXPERIMENT_JOBS = 0
EXPERIMENT_JOBS_COMPLETED = 0
EXPERIMENT_JOBS_FAILED = 0
TEST_FILE_COLUMNS = ["map_seed", "agent_seed", "obstacle", "k_neighbor", "agent"]
TEST_FILE_COLUMNS_DEN520D = ["map_name", "agent_seed", "agent"]
defined_output_prefixes = set()
completed = set()
PBAR = tqdm(total=1)


NAIVE_SETTINGS = [
    (False, False, False),  # online/default,cycle
    (False, True, False),  # feasibility,cycle
    (False, True, True),  # cycle
    # (True, False, False),
    # (True, True, False),       # feasibility
    # (True, True, True),
    # (True, False, False),  # naive feasibility
]

mongo_client = AsyncIOMotorClient()
db = mongo_client["MAPF_DP"]
results_collection = db["results"]
cases_collection = db["cases"]


async def init_db():
    index1 = IndexModel([("experiment", ASCENDING),
                         ("case", ASCENDING)], name="experiment_case", unique=True)
    await results_collection.create_indexes([index1])

    index2 = IndexModel([("case", ASCENDING)], name="case", unique=True)
    await cases_collection.create_indexes([index2])

class CurrentWrapper:
    def __init__(self):
        self.current = 0


def kill_all_process():
    logger.info("Kill all running MAPF_DP processes")
    for proc in psutil.process_iter():
        if proc.name() in ("MAPF_DP", "MAPF_DP.exe"):
            logger.info("Kill process {} with pid {}", proc.name(), proc.pid)
            proc.kill()


def run_program(full_prefix, program_args, timeout):
    start = time.perf_counter()
    p = None
    success = False
    try:
        # logger.debug(" ".join(program_args))
        # p = subprocess.Popen(program_args)
        p = subprocess.Popen(program_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        p.communicate(timeout=timeout)
        success = True
        # p = subprocess.run(program_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
        # logger.debug(p.args)
        # logger.debug(p.returncode)
        # logger.debug(p.stderr.readlines())
        # logger.debug(p.stdout.readlines())
    except subprocess.TimeoutExpired:
        logger.warning('{} timeout', full_prefix)
    except Exception as e:
        logger.error('{} unknown error')
        logger.exception(e)
    try:
        p.kill()
    except:
        pass
    end = time.perf_counter()
    return end - start, success


async def run(args: TestArguments, setup: ExperimentSetup, objective="maximum",
              simulation_seed=0, map_seed=0, map_name="random", agent_seed=0, max_timestep=300,
              map_file: Path = None, task_file: Path = None, agent_skip=0,
              init_tests=False,
              # map_type, objective="maximum",  agents=35, iteration=10,
              # obstacles=90, simulator="online", solver="eecbs", k_neighbor=2, timing="discrete",
              # delay_type="agent", delay_ratio=0.2, delay_start=0, delay_interval=0,
              # naive_feasibility=False, naive_cycle=False, only_cycle=False, feasibility_type=False,
              # timeout=600, init_tests=False
              ):

    global EXPERIMENT_JOBS, EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS_FAILED, PBAR
    EXPERIMENT_JOBS += 1
    PBAR.total = EXPERIMENT_JOBS - EXPERIMENT_JOBS_FAILED
    PBAR.update(0)

    async with args.semaphore:
        output_prefix = setup.get_output_prefix()
        if setup.map == "random":
            map_type = "random"
            cbs_prefix = "%s-%s-32-32-%d-%d-%s-%d-%d-%s" % (
                setup.timing, map_name, setup.obstacles, map_seed, setup.k_neighbor, setup.agents, agent_seed, setup.solver)
            full_prefix = output_prefix + "-%d-%d-%d" % (map_seed, agent_seed, simulation_seed)
        elif setup.map == "warehouse" or setup.map == "mapf":
            map_type = setup.map
            cbs_prefix = "%s-%s-%s-%d-%d-%s" % (
                setup.timing, setup.map, map_name, setup.agents, agent_seed, setup.solver)
            full_prefix = output_prefix + "-%d-%d" % (agent_seed, simulation_seed)
        elif setup.map == "den520d":
            map_type = "graphml"
            cbs_prefix = "%s-%s-%d-%d-%s" % (
                setup.timing, map_name, setup.agents, agent_seed, setup.solver)
            full_prefix = output_prefix + "-%d-%d" % (agent_seed, simulation_seed)
        else:
            assert False

        if init_tests:
            doc_filter = {
                "case": full_prefix,
            }
            collection = cases_collection

        else:
            doc_filter = {
                "experiment": args.experiment_name,
                "case": full_prefix,
            }
            collection = results_collection

        if not args.overwrite:
            doc = await collection.find_one(doc_filter)
            if doc is not None:
                EXPERIMENT_JOBS_COMPLETED += 1
                logger.info('{} skipped ({}/{}/{})', full_prefix, EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS_FAILED,
                            EXPERIMENT_JOBS - EXPERIMENT_JOBS_FAILED)
                PBAR.update()
                return 1

        # async with aiofiles.tempfile.NamedTemporaryFile(prefix="MAPF_DP.", delete=False) as f:
        #     output_file_path = f.name
        #     await f.close()
        # output_file = await
        # output_file_path = output_file
        # output_file, output_file_path = await aiofiles.tempfile.mkstemp(prefix="MAPF_DP.")
        # os.close(output_file)

        output_file = args.result_dir / (full_prefix + ".bson")
        # output_time_file = args.result_dir / (output_prefix + ".time")
        cbs_file = args.result_dir / (cbs_prefix + ".cbs")
        # logger.info(output_file)

        if init_tests:
            cbs_file.unlink(missing_ok=True)

        # if output_prefix not in defined_output_prefixes:
        #     defined_output_prefixes.add(output_prefix)
        #     # if output_file.exists():
        #     if not args.resume:
        #         output_file.unlink(missing_ok=True)
        #         output_time_file.unlink(missing_ok=True)

        # generate arguments
        simulator = setup.simulator
        prioritized_replan = False
        prioritized_opt = False
        online_opt = True
        group_determined = False
        fast_cycle = False
        remove_redundant = "none"
        snapshot_order = "none"
        replan_suboptimality = 1
        replan_nonstop = False
        dep_graph = "boost"

        if setup.simulator.startswith("replan_"):
            simulator = "replan"
            if setup.simulator.startswith("replan_nonstop"):
                replan_nonstop = True
            arr = setup.simulator.split("_")
            if len(arr) > 1:
                replan_suboptimality = float(arr[-1])

        if setup.simulator.startswith("prioritized"):
            simulator = "replan"
            prioritized_replan = True
            if setup.simulator == "prioritized_opt":
                prioritized_opt = True
        elif setup.simulator.startswith("snapshot"):
            simulator = "snapshot"
            if setup.simulator == "snapshot_start":
                snapshot_order = "start"
            elif setup.simulator == "snapshot_end":
                snapshot_order = "end"
            elif setup.simulator == "snapshot_collision":
                snapshot_order = "collision"
        elif setup.simulator.startswith("online"):
            simulator = "online"
            if setup.simulator == "online_remove_redundant_physical":
                remove_redundant = "physical"
            elif setup.simulator == "online_remove_redundant_graph":
                remove_redundant = "graph"
            elif setup.simulator == "online_group":
                group_determined = True
            elif setup.simulator == "online_array":
                dep_graph = "array"
            elif setup.simulator == "online_no_opt":
                online_opt = False
            elif setup.simulator == "online_group_no_opt":
                group_determined = True
                online_opt = False
            elif setup.simulator == "online_array_no_opt":
                dep_graph = "array"
                online_opt = False
            elif setup.simulator == "online_group_array":
                group_determined = True
                dep_graph = "array"
            elif setup.simulator == "online_fast_cycle":
                fast_cycle = True

        program_args = [
            args.program.as_posix(),
            "--map", map_type,
            "--objective", objective,
            "--map-seed", str(map_seed),
            "--map-name", str(map_name),
            "--agent-seed", str(agent_seed),
            "--agent-skip", str(agent_skip),
            "--agents", str(setup.agents),
            "--iteration", "1",
            "--simulation-seed", str(simulation_seed),
            "--solver", setup.solver,
            "--obstacle-ratio", str(setup.obstacles / 100),
            "--simulator", simulator,
            "--k-neighbor", str(setup.k_neighbor),
            "--timing", setup.timing,
            "--delay", setup.delay_type,
            "--delay-ratio", str(setup.delay_ratio),
            "--delay-start", str(setup.delay_start),
            "--delay-interval", str(setup.delay_interval),
            "--max-timestep", str(max_timestep),
            "--output-format", "bson",
            "--output", output_file.as_posix(),
            # "--time-output", output_time_file.as_posix(),
            "--suboptimality", str(args.suboptimality),
            "--snapshot-order", snapshot_order,
            "--remove-redundant", remove_redundant,
            "--replan-suboptimality", str(replan_suboptimality),
            "--dep-graph", dep_graph,
        ]
        # if map_type == "hardcoded":
        #     program_args.append("--all")
        if not init_tests:
            program_args.append("-v")
        if setup.feasibility != "h":
            program_args.append("--naive-feasibility")
        if setup.cycle != "h":
            program_args.append("--naive-cycle")
        if setup.cycle == "o":
            program_args.append("--only-cycle")
        # if feasibility_type:
        #     args.append("--feasibility-type")
        if prioritized_replan:
            program_args.append("--prioritized-replan")
        if prioritized_opt:
            program_args.append("--prioritized-opt")
        if online_opt:
            program_args.append("--online-opt")
        if group_determined:
            program_args.append("--group-determined")
        if fast_cycle:
            program_args.append("--fast-cycle")
        if replan_nonstop:
            program_args.append("--replan-nonstop")

        if map_file:
            program_args.append("--map-file")
            program_args.append(map_file.as_posix())
        if task_file:
            program_args.append("--task-file")
            program_args.append(task_file.as_posix())

        # logger.info("{}", " ".join(program_args))
        elapsed_seconds, success = await asyncio.get_event_loop().run_in_executor(
            args.pool, run_program, full_prefix, program_args, args.timeout)

        decoded_data = {}
        if success:
            try:
                async with aiofiles.open(str(output_file), "rb") as f:
                    output_data = await f.read()
                decoded_data = bson.decode(output_data)
            except Exception as e:
                logger.error("{} {}", e.__class__, str(e))
                success = False

        result = 1
        if not decoded_data.get("result", {}).get("success", False):
            success = False

        seeds = {
            "map_seed": map_seed,
            "agent_seed": agent_seed,
            "simulation_seed": simulation_seed,
        }

        if init_tests:
            try:
                if not success:
                    raise Exception()
                with cbs_file.open() as file:
                    line = file.readline().strip()
                    if len(line) == 0:
                        # logger.error("{}", cbs_file)
                        raise Exception()
                # with output_file.open() as file:
                #     line = file.readline().strip()
                #     if len(line) == 0:
                #         # logger.error("{}", output_file)
                #         raise Exception()
            except Exception as e:
                # logger.exception(e)
                result = 0
            if result == 1:
                doc = {
                    "case": full_prefix,
                    "setup": setup.dict(),
                    "seeds": seeds,
                    "success": success,
                    **decoded_data,
                }
            else:
                doc = None

        else:
            doc = {
                "experiment": args.experiment_name,
                "case": full_prefix,
                "setup": setup.dict(),
                "seeds": seeds,
                "success": success,
                **decoded_data,
            }

        if doc is not None:
            if args.overwrite:
                upsert_result = await collection.replace_one(doc_filter, doc, upsert=True)
                logger.info("upsert: {}", upsert_result.raw_result)
            else:
                insert_result = await collection.insert_one(doc)
                logger.info("insert: {}", insert_result.inserted_id)

        if result == 1 and success:
            EXPERIMENT_JOBS_COMPLETED += 1
            PBAR.update(1)
            logger.info('{} success ({}/{}/{}) in {} seconds', full_prefix,
                        EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS_FAILED,
                        EXPERIMENT_JOBS - EXPERIMENT_JOBS_FAILED, elapsed_seconds)
        else:
            EXPERIMENT_JOBS_FAILED += 1
            PBAR.total = EXPERIMENT_JOBS - EXPERIMENT_JOBS_FAILED
            PBAR.update(0)
            logger.info('{} failed ({}/{}/{}) in {} seconds', full_prefix,
                        EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS_FAILED,
                        EXPERIMENT_JOBS - EXPERIMENT_JOBS_FAILED, elapsed_seconds)
        return result


async def do_init_tests(args: TestArguments):
    # all_tests = []
    logger.info("Initialize tests")
    # test_file = args.result_dir / "tests.csv"
    # if test_file.exists():
    #     df = pd.read_csv(test_file, header=None)
    #     df.columns = TEST_FILE_COLUMNS
    #     group = df.groupby(["map_seed", "obstacle", "k_neighbor", "agent"])
    #     df["count"] = group["agent_seed"].transform(len)
    #     idx = df.groupby(["map_seed", "obstacle", "k_neighbor", "agent"])["agent_seed"].transform(max) == df[
    #         'agent_seed']
    #     df = df[idx]
    # else:
    #     df = pd.DataFrame(columns=TEST_FILE_COLUMNS)

    # logger.info(df)
    # test_file.unlink(missing_ok=True)

    async def init_case(map_seed, agent_seed, obstacles, k_neighbor, agents):
        setup = ExperimentSetup(
            timing=args.timing, map="random", map_name="random",
            solver=args.solver, simulator="default", obstacles=obstacles,
            k_neighbor=k_neighbor, agents=agents, delay_type="agent",
            delay_ratio=0, delay_start=0, delay_interval=0,
            feasibility="h", cycle="h",
        )
        return await run(args, setup, map_seed=map_seed, agent_seed=agent_seed, init_tests=True)

    async def init_map(map_seed, obstacle, k_neighbor, agent):
        _current = CurrentWrapper()
        # row = df.loc[(df["map_seed"] == map_seed) & (df["obstacle"] == obstacle) &
        #              (df["k_neighbor"] == k_neighbor) & (df["agent"] == agent)]
        # if len(row) > 0:
        #     row = row.iloc[0]
        #     _current.current = row["agent_seed"] + 1
        #     completed_seeds = min(args.agent_seeds, row["count"])
        # else:
        #     completed_seeds = 0
        # if completed_seeds > 0:
        #     setup = ExperimentSetup(
        #         timing=args.timing, map="random", map_name="random",
        #         solver=args.solver, simulator="default", obstacles=obstacle,
        #         k_neighbor=k_neighbor, agents=agent, delay_type="agent",
        #         delay_ratio=0, delay_start=0, delay_interval=0,
        #         feasibility="h", cycle="h",
        #     )
        #     logger.info('{} tests skipped for {}-{}-*', completed_seeds, setup.get_output_prefix(), map_seed)
        agent_seeds = args.agent_seeds

        async def init_agent():
            while True:
                agent_seed = _current.current
                _current.current += 1
                result = await init_case(map_seed, agent_seed, obstacle, k_neighbor, agent)
                if result == 1:
                    # with test_file.open("a") as file:
                    #     file.write("%d,%d,%d,%d,%d\n" % (map_seed, agent_seed, obstacle, k_neighbor, agent))
                    break

        tasks = []
        for i in range(agent_seeds):
            tasks.append(init_agent())

        await asyncio.gather(*tasks)

        # while completed < agent_seeds:
        #     tasks = []
        #     for i in range(current, current + agent_seeds - completed):
        #         tasks.append(init_case(map_seed, i, obstacle, k_neighbor, agent))
        #     results = await asyncio.gather(*tasks)
        #     for i, result in enumerate(results):
        #         if result == 1:
        #             all_tests.append((map_seed, current + i, obstacle, k_neighbor, agent))
        #     current += len(results)
        #     completed += sum(results)

    map_tasks = []
    for _map_seed in range(args.map_seeds):
        for _obstacle in args.obstacles:
            for _agent in args.agents:
                for _k_neighbor in args.k_neighbors:
                    map_tasks.append(init_map(_map_seed, _obstacle, _k_neighbor, _agent))
    await asyncio.gather(*map_tasks)

    # test_file = result_dir / "tests.csv"
    # with test_file.open("w") as file:
    #     for test in all_tests:
    #         file.write(",".join(map(lambda x: str(x), test)) + "\n")


async def do_tests(args: TestArguments):
    async def init_case(map_seed, agent_seed, obstacles, k_neighbor, agents, simulator, delay_ratio, delay_interval,
                        naive_feasibility, naive_cycle, only_cycle, simulation_seed):
        if not naive_feasibility:
            feasibility = "h"
        else:
            feasibility = "n"
        if not naive_cycle:
            cycle = "h"
        else:
            if only_cycle:
                cycle = "o"
            else:
                cycle = "n"

        setup = ExperimentSetup(
            timing=args.timing, map="random", map_name="random",
            solver=args.solver, simulator=simulator, obstacles=obstacles,
            k_neighbor=k_neighbor, agents=agents, delay_type="agent",
            delay_ratio=delay_ratio, delay_start=0, delay_interval=delay_interval,
            feasibility=feasibility, cycle=cycle,
        )
        return await run(args, setup, map_seed=map_seed, agent_seed=agent_seed, simulation_seed=simulation_seed,
                         init_tests=False)

    test_file = args.result_dir / "tests.csv"
    df = pd.read_csv(test_file, header=None)
    df.columns = TEST_FILE_COLUMNS

    completed_file = args.result_dir / "completed.csv"
    if completed_file.exists():
        with completed_file.open("r") as f:
            for line in f.readlines():
                completed.add(line.strip())

    tasks = []
    for _map_seed in range(args.map_seeds):
        for _obstacle in args.obstacles:
            for _agent in args.agents:
                for _k_neighbor in args.k_neighbors:
                    map_name = "random-32-32-%d-%d" % (_obstacle, _k_neighbor)
                    doc_filter = {
                        "setup.timing": args.timing,
                        "setup.map": args.map,
                        "setup.map_name": map_name,
                        "setup.obstacles": _obstacle,
                        "setup.k_neighbor": _k_neighbor,
                        "setup.agents": _agent,
                        "seeds.map_seed": _map_seed,
                    }
                    agent_seeds_arr = [document["seeds"]["agent_seed"] async for document in cases_collection.find(doc_filter)]
                    agent_seeds_arr = sorted(agent_seeds_arr)
                    if len(agent_seeds_arr) < args.agent_seeds:
                        logger.warning("{} {} {} {} no enough seeds", _map_seed, _obstacle, _k_neighbor, _agent)
                    for i in range(min(len(agent_seeds_arr), args.agent_seeds)):
                        _agent_seed = agent_seeds_arr[i]
                        for _delay_ratio in args.delay_ratios:
                            for _delay_interval in args.delay_intervals:
                                for _simulator in args.simulators:
                                    if args.naive and _simulator.startswith("online"):
                                        naive_settings = NAIVE_SETTINGS
                                    else:
                                        naive_settings = [(False, False, False)]
                                    for (_naive_feasibility, _naive_cycle, _only_cycle) in naive_settings:
                                        for _simulation_seed in range(args.iteration):
                                            tasks.append(
                                                init_case(_map_seed, _agent_seed, _obstacle, _k_neighbor, _agent,
                                                          _simulator, _delay_ratio, _delay_interval,
                                                          _naive_feasibility, _naive_cycle, _only_cycle,
                                                          _simulation_seed))
    await asyncio.gather(*tasks)


async def do_init_tests_den520d(args: TestArguments):
    # all_tests = []
    logger.info("Initialize tests")
    # test_file = args.result_dir / f"tests_{args.map}.csv"
    # if test_file.exists():
    #     df = pd.read_csv(test_file, header=None)
    #     df.columns = TEST_FILE_COLUMNS_DEN520D
    #     df.sort_values(by=["map_name", "agent", "agent_seed"], inplace=True)
    #     df.to_csv(test_file, header=False, index=False)
    #     group = df.groupby(["map_name", "agent"])
    #     df["count"] = group["agent_seed"].transform(len)
    #     idx = df.groupby(["map_name", "agent"])["agent_seed"].transform(max) == df[
    #         'agent_seed']
    #     df = df[idx]
    # else:
    #     df = pd.DataFrame(columns=TEST_FILE_COLUMNS_DEN520D)

    # logger.info(df)
    # test_file.unlink(missing_ok=True)

    if args.timing == "discrete":
        simulator = "default"
    else:
        simulator = "snapshot_start"

    async def init_case(map_name, agent_seed, agents, agents_per_task_file):
        setup = ExperimentSetup(
            timing=args.timing, map=args.map, map_name=map_name, solver=args.solver,
            simulator=simulator, agents=agents, delay_type="agent",
            delay_ratio=0, delay_start=0, delay_interval=0,
            feasibility="h", cycle="h",
        )

        if args.map == "den520d" or args.map == "mapf":
            # task_per_task_file = int(agents_per_task_file / agents)
            # task_file_id = int(agent_seed / task_per_task_file)
            # agent_skip = agents * (agent_seed % task_per_task_file)
            task_file_id = agent_seed
            agent_skip = 0

            if args.map == "den520d":
                base_dir = args.maps_dir / "roadmaps" / map_name
                map_file = base_dir / "map.xml"
                task_file = base_dir / f"{task_file_id + 1}_task.xml"
            elif args.map == "mapf":
                base_task_dir = args.mapf_maps_dir / "scen-even"
                map_file = args.mapf_maps_dir / "map" / map_name
                task_file = base_task_dir / f"{map_name}-even-{task_file_id + 1}.scen"
            else:
                assert False

            return await run(args, setup, agent_seed=agent_seed, map_name=map_name,
                             map_file=map_file, task_file=task_file, agent_skip=agent_skip,
                             max_timestep=10000, init_tests=True)

        elif args.map == "warehouse":
            return await run(args, setup, agent_seed=agent_seed, map_name=map_name,
                             max_timestep=10000, init_tests=True)

    async def init_map(map_name, agent):
        _current = CurrentWrapper()
        # row = df.loc[(df["map_name"] == map_name) & (df["agent"] == agent)]
        # if len(row) > 0:
        #     row = row.iloc[0]
        #     _current.current = row["agent_seed"] + 1
        #     completed_seeds = min(args.agent_seeds, row["count"])
        # else:
        #     completed_seeds = 0
        # if completed_seeds > 0:
        #     setup = ExperimentSetup(
        #         timing=args.timing, map=args.map, map_name=map_name, solver=args.solver,
        #         simulator=simulator, agents=agent, delay_type="agent",
        #         delay_ratio=0, delay_start=0, delay_interval=0,
        #         feasibility="h", cycle="h",
        #     )
        #     logger.info('{} tests skipped for {}-{}-*', completed_seeds, setup.get_output_prefix(), map_name)
        # agent_seeds = args.agent_seeds - completed_seeds
        agent_seeds = args.agent_seeds

        if args.map == "den520d":
            agents_per_task_file = 100
        elif args.map == "mapf":
            base_task_dir = args.mapf_maps_dir / "scen-even"
            task_file = base_task_dir / f"{map_name}-even-1.scen"
            with task_file.open("r") as f:
                agents_per_task_file = len(f.readlines()) - 2
        else:
            agents_per_task_file = 0

        if args.map == "den520d" or args.map == "mapf":
            # task_per_task_file = int(agents_per_task_file / agent)
            # max_agent_seed = 25 * task_per_task_file
            max_agent_seed = 25
        else:
            max_agent_seed = math.inf

        async def init_agent():
            while True:
                agent_seed = _current.current
                _current.current += 1
                if agent_seed >= max_agent_seed:
                    break
                result = await init_case(map_name, agent_seed, agent, agents_per_task_file)
                if result == 1:
                    # with test_file.open("a") as file:
                    #     file.write("%s,%d,%d\n" % (map_name, agent_seed, agent))
                    break

        tasks = []
        for i in range(agent_seeds):
            tasks.append(init_agent())

        await asyncio.gather(*tasks)

    map_tasks = []
    for _map_name in args.map_names:
        for _agent in args.agents:
            map_tasks.append(init_map(_map_name, _agent))
    await asyncio.gather(*map_tasks)


async def do_tests_den520d(args: TestArguments):
    async def init_case(map_name, agent_seed, agents, simulator, delay_ratio, delay_interval,
                        naive_feasibility, naive_cycle, only_cycle, agents_per_task_file, simulation_seed):
        if not naive_feasibility:
            feasibility = "h"
        else:
            feasibility = "n"
        if not naive_cycle:
            cycle = "h"
        else:
            if only_cycle:
                cycle = "o"
            else:
                cycle = "n"

        setup = ExperimentSetup(
            timing=args.timing, map=args.map, map_name=map_name, solver=args.solver,
            simulator=simulator, agents=agents, delay_type="agent",
            delay_ratio=delay_ratio, delay_start=0, delay_interval=delay_interval,
            feasibility=feasibility, cycle=cycle,
        )

        if args.map == "den520d" or args.map == "mapf":
            # task_per_task_file = int(agents_per_task_file / agents)
            # task_file_id = int(agent_seed / task_per_task_file)
            # agent_skip = agents * (agent_seed % task_per_task_file)
            task_file_id = agent_seed
            agent_skip = 0

            if args.map == "den520d":
                base_dir = args.maps_dir / "roadmaps" / map_name
                map_file = base_dir / "map.xml"
                task_file = base_dir / f"{task_file_id + 1}_task.xml"
            elif args.map == "mapf":
                base_task_dir = args.mapf_maps_dir / "scen-even"
                map_file = args.mapf_maps_dir / "map" / map_name
                task_file = base_task_dir / f"{map_name}-even-{task_file_id + 1}.scen"
            else:
                assert False

            return await run(args, setup, agent_seed=agent_seed, simulation_seed=simulation_seed, map_name=map_name,
                             map_file=map_file, task_file=task_file, agent_skip=agent_skip, max_timestep=10000,
                             init_tests=False)
        elif args.map == "warehouse":
            return await run(args, setup, agent_seed=agent_seed, simulation_seed=simulation_seed, map_name=map_name,
                             max_timestep=10000, init_tests=False)

    # test_file = args.result_dir / f"tests_{args.map}.csv"
    # df = pd.read_csv(test_file, header=None)
    # df.columns = TEST_FILE_COLUMNS_DEN520D
    #
    # completed_file = args.result_dir / f"completed.csv"
    # if completed_file.exists():
    #     with completed_file.open("r") as f:
    #         for line in f.readlines():
    #             completed.add(line.strip())

    tasks = []
    for _map_name in args.map_names:
        if args.map == "den520d":
            agents_per_task_file = 100
        elif args.map == "mapf":
            base_task_dir = args.mapf_maps_dir / "scen-even"
            task_file = base_task_dir / f"{_map_name}-even-1.scen"
            with task_file.open("r") as f:
                agents_per_task_file = len(f.readlines()) - 2
        else:
            agents_per_task_file = 0

        for _agent in args.agents:
            doc_filter = {
                "setup.timing": args.timing,
                "setup.map": args.map,
                "setup.map_name": _map_name,
                "setup.agents": _agent,
            }
            agent_seeds_arr = [document["seeds"]["agent_seed"] async for document in cases_collection.find(doc_filter)]
            if len(agent_seeds_arr) < args.agent_seeds:
                logger.warning("{} {} no enough seeds", _map_name, _agent)
            for i in range(min(len(agent_seeds_arr), args.agent_seeds)):
                _agent_seed = agent_seeds_arr[i]
                for _delay_ratio in args.delay_ratios:
                    for _delay_interval in args.delay_intervals:
                        for _simulator in args.simulators:
                            if args.naive and _simulator.startswith("online"):
                                naive_settings = NAIVE_SETTINGS
                            else:
                                naive_settings = [(False, False, False)]
                            for (_naive_feasibility, _naive_cycle, _only_cycle) in naive_settings:
                                for _simulation_seed in range(args.iteration):
                                    tasks.append(
                                        init_case(_map_name, _agent_seed, _agent,
                                                  _simulator, _delay_ratio, _delay_interval,
                                                  _naive_feasibility, _naive_cycle, _only_cycle,
                                                  agents_per_task_file, _simulation_seed))
    await asyncio.gather(*tasks)


@app_command("test")
@click.option("--map-seeds", type=int, default=10)
@click.option("--map-names", type=str, default="", callback=validate_list(str))
@click.option("--agent-seeds", type=int, default=10)
@click.option("--iteration", type=int, default=10)
@click.option("-t", "--timeout", type=int, default=10)
@click.option("--suboptimality", type=float, default=1.1)
@click.option("--init-tests", type=bool, default=False, is_flag=True)
@click.option("-j", "--jobs", type=int, default=multiprocessing.cpu_count)
@click.option("--naive", type=bool, default=False, is_flag=True)
@click.option("--overwrite", type=bool, default=False, is_flag=True)
@click.option("--experiment-name", type=str, default="test1")
@click.pass_context
@asyncio_wrapper
async def main(ctx, map_seeds, map_names, agent_seeds, iteration, timeout, suboptimality, init_tests, jobs, naive,
               overwrite, experiment_name):
    program = project_root / "cmake-build-relwithdebinfo"
    if platform.system() == "Windows":
        program = program / "MAPF_DP.exe"
    else:
        program = program / "MAPF_DP"
    maps_dir = project_root / "maps"
    mapf_maps_dir = project_root / "MAPF-benchmark"
    result_dir = project_root / "result"
    result_dir.mkdir(parents=True, exist_ok=True)
    os.chdir(result_dir)

    if ctx.obj.timing == "discrete":
        solver = "eecbs"
    else:
        solver = "ccbs"

    pool = concurrent.futures.ProcessPoolExecutor(max_workers=jobs)
    semaphore = asyncio.Semaphore(jobs * 2)
    args = TestArguments(
        **ctx.obj.__dict__,
        experiment_name=experiment_name,
        map_seeds=map_seeds,
        map_names=map_names,
        agent_seeds=agent_seeds,
        iteration=iteration,
        timeout=timeout,
        solver=solver,
        suboptimality=suboptimality,
        program=program,
        maps_dir=maps_dir,
        mapf_maps_dir=mapf_maps_dir,
        result_dir=result_dir,
        pool=pool,
        semaphore=semaphore,
        naive=naive,
        overwrite=overwrite,
    )
    logger.info(args)
    logger.info("Running with {} jobs", jobs)
    # click.echo(args)

    kill_all_process()
    await init_db()

    if args.map == "random":
        if init_tests:
            await do_init_tests(args)
        else:
            await do_tests(args)
    elif args.map == "den520d" or args.map == "warehouse" or args.map == "mapf":
        if args.map == "warehouse":
            args.map_names = ["22-57"]
        if init_tests:
            await do_init_tests_den520d(args)
        else:
            await do_tests_den520d(args)
    PBAR.close()


if __name__ == '__main__':
    main()
