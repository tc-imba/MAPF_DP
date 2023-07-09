import asyncio
import os
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

from experiment.app import app_command, AppArguments
from experiment.utils import asyncio_wrapper, project_root, ExperimentSetup
from experiment.logger import logger

@dataclasses.dataclass
class TestArguments(AppArguments):
    map_seeds: int
    agent_seeds: int
    iteration: int
    timeout: int
    suboptimality: float
    solver: str
    program: Path
    result_dir: Path


workers = multiprocessing.cpu_count()
# workers = 1
pool = concurrent.futures.ProcessPoolExecutor(max_workers=workers)
EXPERIMENT_JOBS = 0
EXPERIMENT_JOBS_COMPLETED = 0
defined_output_prefixes = set()
completed = set()
PBAR = tqdm(total=1)

NAIVE_SETTINGS = [
    (False, False, False),  # online/default,cycle
    # (False, True, False),  # feasibility,cycle
    # (False, True, True),  # cycle
    # (True, False, False),
    # (True, True, False),       # feasibility
    # (True, True, True),
]

def kill_all_process():
    logger.info("Kill all running MAPF_DP processes")
    for proc in psutil.process_iter():
        if proc.name() in ("MAPF_DP", "MAPF_DP.exe"):
            logger.info("Kill process {} with pid {}", proc.name(), proc.pid)
            proc.kill()

def run_program(full_prefix, program_args, timeout):
    p = None
    try:
        # logger.debug(" ".join(program_args))
        p = subprocess.Popen(program_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        p.communicate(timeout=timeout)
        # p = subprocess.run(program_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
        # logger.debug(p.args)
        # logger.debug(p.returncode)
        # logger.debug(p.stderr)
        # logger.debug(p.stdout)
    except subprocess.TimeoutExpired:
        logger.warning('{} timeout', full_prefix)
    except Exception as e:
        logger.error('{} unknown error')
        logger.exception(e)
    try:
        p.kill()
    except:
        pass


async def run(args: TestArguments, setup: ExperimentSetup,
              map_type="random", objective="maximum",
              iteration=10, map_seed=0, agent_seed=0, init_tests=False,
              # map_type, objective="maximum",  agents=35, iteration=10,
              # obstacles=90, simulator="online", solver="eecbs", k_neighbor=2, timing="discrete",
              # delay_type="agent", delay_ratio=0.2, delay_start=0, delay_interval=0,
              # naive_feasibility=False, naive_cycle=False, only_cycle=False, feasibility_type=False,
              # timeout=600, init_tests=False
              ):
    global EXPERIMENT_JOBS, EXPERIMENT_JOBS_COMPLETED, PBAR
    EXPERIMENT_JOBS += 1
    PBAR.total = EXPERIMENT_JOBS
    PBAR.refresh()

    output_prefix = setup.get_output_prefix()
    full_prefix = output_prefix + "-%d-%d" % (map_seed, agent_seed)
    if init_tests:
        output_prefix = full_prefix
    elif full_prefix in completed:
        EXPERIMENT_JOBS_COMPLETED += 1
        logger.info('{} skipped ({}/{})', full_prefix, EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS)
        return 1
    output_file = args.result_dir / (output_prefix + ".csv")
    output_time_file = args.result_dir / (output_prefix + ".time")

    cbs_prefix = "%s-random-30-30-%d-%d-%s-%d-%d-%s" % (
        setup.timing, setup.obstacles, map_seed, setup.k_neighbor, setup.agents, agent_seed, setup.solver)
    cbs_file = args.result_dir / (cbs_prefix + ".cbs")

    if init_tests:
        cbs_file.unlink(missing_ok=True)

    if output_prefix not in defined_output_prefixes:
        defined_output_prefixes.add(output_prefix)
        # if output_file.exists():
        output_file.unlink(missing_ok=True)
        output_time_file.unlink(missing_ok=True)

    # generate arguments
    simulator = setup.simulator
    prioritized_replan = False
    prioritized_opt = False
    if setup.simulator.startswith("prioritized"):
        simulator = "replan"
        prioritized_replan = True
        if setup.simulator == "prioritized_opt":
            prioritized_opt = True

    program_args = [
        args.program.as_posix(),
        "--map", map_type,
        "--objective", objective,
        "--map-seed", str(map_seed),
        "--agent-seed", str(agent_seed),
        "--agents", str(setup.agents),
        "--iteration", str(iteration),
        "--solver", setup.solver,
        "--obstacles", str(setup.obstacles),
        "--simulator", simulator,
        "--k-neighbor", str(setup.k_neighbor),
        "--timing", setup.timing,
        "--delay", setup.delay_type,
        "--delay-ratio", str(setup.delay_ratio),
        "--delay-start", str(setup.delay_start),
        "--delay-interval", str(setup.delay_interval),
        "--output", output_file.as_posix(),
        "--time-output", output_time_file.as_posix(),
        "--suboptimality", str(args.suboptimality),
    ]
    if map_type == "hardcoded":
        program_args.append("--all")
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

    await asyncio.get_event_loop().run_in_executor(pool, run_program, full_prefix, program_args, args.timeout)
    EXPERIMENT_JOBS_COMPLETED += 1
    PBAR.update()
    PBAR.refresh()

    result = 1
    if init_tests:
        try:
            with cbs_file.open() as file:
                line = file.readline().strip()
                if len(line) == 0:
                    raise Exception()
            with output_file.open() as file:
                line = file.readline().strip()
                if len(line) == 0:
                    raise Exception()
        except:
            result = 0

    if result == 1:
        logger.info('{} completed ({}/{})', full_prefix, EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS)
        completed.add(full_prefix)
        completed_file = args.result_dir / "completed.csv"
        with completed_file.open("a") as f:
            f.write("%s\n" % full_prefix)
    else:
        logger.info('{} failed ({}/{})', full_prefix, EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS)

    return result


async def do_init_tests(args: TestArguments):
    # all_tests = []
    logger.info("Initialize tests")
    test_file = args.result_dir / "tests.csv"
    test_file.unlink(missing_ok=True)

    async def init_case(map_seed, agent_seed, obstacles, k_neighbor, agents):
        setup = ExperimentSetup(
            timing=args.timing, solver=args.solver, simulator="default", obstacles=obstacles,
            k_neighbor=k_neighbor, agents=agents, delay_type="agent",
            delay_ratio=0, delay_start=0, delay_interval=0,
            feasibility="h", cycle="h",
        )
        return await run(args, setup, map_seed=map_seed, agent_seed=agent_seed, iteration=1, init_tests=True)

    async def init_map(map_seed, obstacle, k_neighbor, agent):

        class CurrentWrapper:
            def __init__(self):
                self.current = 0

        _current = CurrentWrapper()

        async def init_agent():
            while True:
                agent_seed = _current.current
                _current.current += 1
                result = await init_case(map_seed, agent_seed, obstacle, k_neighbor, agent)
                if result == 1:
                    with test_file.open("a") as file:
                        file.write("%d,%d,%d,%d,%d\n" % (map_seed, agent_seed, obstacle, k_neighbor, agent))
                    break

        tasks = []
        for i in range(args.agent_seeds):
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
                        naive_feasibility, naive_cycle, only_cycle):
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
            timing=args.timing, solver=args.solver, simulator=simulator, obstacles=obstacles,
            k_neighbor=k_neighbor, agents=agents, delay_type="agent",
            delay_ratio=delay_ratio, delay_start=0, delay_interval=delay_interval,
            feasibility=feasibility, cycle=cycle,
        )
        return await run(args, setup, map_seed=map_seed, agent_seed=agent_seed, iteration=args.iteration,
                         init_tests=False)

    test_file = args.result_dir / "tests.csv"
    df = pd.read_csv(test_file, header=None)
    df.columns = ["map_seed", "agent_seed", "obstacle", "k_neighbor", "agent"]

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
                    df_temp = df[(df["map_seed"] == _map_seed) & (df["obstacle"] == _obstacle) & (
                            df["k_neighbor"] == _k_neighbor) & (df["agent"] == _agent)]
                    df_temp.sort_values(by="agent_seed")
                    if len(df_temp) < args.agent_seeds:
                        logger.warning("{} {} {} {} no enough seeds",_map_seed, _obstacle, _k_neighbor, _agent)
                    for i in range(min(len(df_temp), args.agent_seeds)):
                        _agent_seed = df_temp["agent_seed"].iloc[i]
                        for _delay_ratio in args.delay_ratios:
                            for _delay_interval in args.delay_intervals:
                                for _simulator in args.simulators:
                                    if _simulator != "online":
                                        naive_settings = [(False, False, False)]
                                    else:
                                        naive_settings = NAIVE_SETTINGS
                                    for (_naive_feasibility, _naive_cycle, _only_cycle) in naive_settings:
                                        tasks.append(
                                            init_case(_map_seed, _agent_seed, _obstacle, _k_neighbor, _agent,
                                                      _simulator, _delay_ratio, _delay_interval,
                                                      _naive_feasibility, _naive_cycle, _only_cycle))
    await asyncio.gather(*tasks)


@app_command("test")
@click.option("--map-seeds", type=int, default=10)
@click.option("--agent-seeds", type=int, default=10)
@click.option("--iteration", type=int, default=10)
@click.option("-t", "--timeout", type=int, default=600)
@click.option("--suboptimality", type=float, default=1)
@click.option("--init-tests", type=bool, default=False, is_flag=True)
@click.pass_context
@asyncio_wrapper
async def main(ctx, map_seeds, agent_seeds, iteration, timeout, suboptimality, init_tests):
    program = project_root / "cmake-build-relwithdebinfo"
    if platform.system() == "Windows":
        program = program / "MAPF_DP.exe"
    else:
        program = program / "MAPF_DP"
    result_dir = project_root / "result"
    result_dir.mkdir(parents=True, exist_ok=True)
    os.chdir(result_dir)

    if ctx.obj.timing == "discrete":
        solver = "eecbs"
    else:
        solver = "ccbs"

    args = TestArguments(
        **ctx.obj.__dict__,
        map_seeds=map_seeds,
        agent_seeds=agent_seeds,
        iteration=iteration,
        timeout=timeout,
        solver=solver,
        suboptimality=suboptimality,
        program=program,
        result_dir=result_dir,
    )
    logger.info(args)
    # click.echo(args)

    kill_all_process()
    if init_tests:
        await do_init_tests(args)
    else:
        await do_tests(args)

    PBAR.close()

if __name__ == '__main__':
    main()
