import asyncio
import os
import click
import subprocess
import platform
from pathlib import Path
import multiprocessing
import concurrent.futures
import pandas as pd

from experiment.utils import asyncio_wrapper, validate_list

project_root = Path(__file__).parent.parent
program = project_root / "cmake-build-relwithdebinfo"
if platform.system() == "Windows":
    program = program / "MAPF_DP.exe"
else:
    program = program / "MAPF_DP"
result_dir = project_root / "result"
result_dir.mkdir(parents=True, exist_ok=True)

# workers = multiprocessing.cpu_count()
workers = 1
pool = concurrent.futures.ProcessPoolExecutor(max_workers=workers)
EXPERIMENT_JOBS = 0
EXPERIMENT_JOBS_COMPLETED = 0
result_files = set()
completed = set()

NAIVE_SETTINGS = [
    (False, False, False),  # online/default,cycle
    (False, True, False),  # feasibility,cycle
    (False, True, True),  # cycle
    # (True, False, False),
    # (True, True, False),       # feasibility
    # (True, True, True),
]


def run_program(args, timeout):
    p = None
    try:
        p = subprocess.run([program.as_posix(), *args], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
        # print(program.as_posix(), ' '.join(args))
        # print(p.args)
        # print(p.returncode)
        # print(p.stderr)
        # print(p.stdout)
    except asyncio.TimeoutError:
        print('timeout!')
    except Exception as e:
        print(e)
    try:
        p.kill()
    except:
        pass


async def run(map_type, objective="maximum", map_seed=0, agent_seed=0, agents=35, iteration=10,
              obstacles=90, simulator="online", solver="eecbs", k_neighbor=2, timing="discrete",
              delay_type="agent", delay_ratio=0.2, delay_start=0, delay_interval=0,
              naive_feasibility=False, naive_cycle=False, only_cycle=False, feasibility_type=False,
              timeout=600, init_tests=False):
    global EXPERIMENT_JOBS, EXPERIMENT_JOBS_COMPLETED
    EXPERIMENT_JOBS += 1

    output_prefix = "%s-%s-%d-%d-%d-%s-%s-%d-%d-%s-%s" % (
        timing, simulator, obstacles, k_neighbor, agents, delay_type, delay_ratio, delay_start, delay_interval,
        naive_feasibility and "n" or "h",
        only_cycle and "o" or (naive_cycle and "n" or "h"),
    )
    full_prefix = output_prefix + "-%d-%d" % (map_seed, agent_seed)
    if init_tests:
        output_prefix = full_prefix
    elif full_prefix in completed:
        EXPERIMENT_JOBS_COMPLETED += 1
        print('%s skipped (%d/%d)' % (full_prefix, EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS))
        return 1
    output_file = result_dir / (output_prefix + ".csv")

    cbs_prefix = "%s-random-30-30-%d-%d-%s-%d-%d-%s" % (
        timing, obstacles, map_seed, k_neighbor, agents, agent_seed, solver)
    cbs_file = result_dir / (cbs_prefix + ".cbs")

    if init_tests:
        cbs_file.unlink(missing_ok=True)

    if output_prefix not in result_files:
        result_files.add(output_prefix)
        # if output_file.exists():
        output_file.unlink(missing_ok=True)

    # generate arguments
    if simulator == "prioritized":
        simulator = "replan"
        prioritized_replan = True
    else:
        prioritized_replan = False

    args = [
        # program,
        "--map", map_type,
        "--objective", objective,
        "--map-seed", str(map_seed),
        "--agent-seed", str(agent_seed),
        "--agents", str(agents),
        "--iteration", str(iteration),
        "--solver", solver,
        "--obstacles", str(obstacles),
        "--simulator", simulator,
        "--k-neighbor", str(k_neighbor),
        "--timing", timing,
        "--delay", delay_type,
        "--delay-ratio", str(delay_ratio),
        "--delay-start", str(delay_start),
        "--delay-interval", str(delay_interval),
        "--output", output_file.as_posix(),
    ]
    if map_type == "hardcoded":
        args.append("--all")
    if naive_feasibility:
        args.append("--naive-feasibility")
    if naive_cycle:
        args.append("--naive-cycle")
    if only_cycle:
        args.append("--only-cycle")
    if feasibility_type:
        args.append("--feasibility-type")
    if prioritized_replan:
        args.append("--prioritized-replan")

    await asyncio.get_event_loop().run_in_executor(pool, run_program, args, timeout)
    EXPERIMENT_JOBS_COMPLETED += 1

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
        print('%s completed (%d/%d)' % (full_prefix, EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS))
        completed.add(full_prefix)
        completed_file = result_dir / "completed.csv"
        with completed_file.open("a") as f:
            f.write("%s\n" % full_prefix)
    else:
        print('%s failed (%d/%d)' % (full_prefix, EXPERIMENT_JOBS_COMPLETED, EXPERIMENT_JOBS))

    return result


async def do_init_tests(map_seeds, agent_seeds, obstacles, k_neighbors, agents, timeout):
    # all_tests = []

    test_file = result_dir / "tests.csv"
    test_file.unlink(missing_ok=True)

    async def init_case(map_seed, agent_seed, obstacle, k_neighbor, agent):
        return await run("random", map_seed=map_seed, agent_seed=agent_seed, obstacles=obstacle, agents=agent,
                         simulator="default", k_neighbor=k_neighbor, iteration=1,
                         delay_type="agent", delay_ratio=0, delay_start=0, delay_interval=0,
                         timeout=timeout, init_tests=True)

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
    for _map_seed in range(map_seeds):
        for _obstacle in obstacles:
            for _agent in agents:
                for _k_neighbor in k_neighbors:
                    map_tasks.append(init_map(_map_seed, _obstacle, _k_neighbor, _agent))
    await asyncio.gather(*map_tasks)

    # test_file = result_dir / "tests.csv"
    # with test_file.open("w") as file:
    #     for test in all_tests:
    #         file.write(",".join(map(lambda x: str(x), test)) + "\n")


async def do_tests(map_seeds, agent_seeds, iteration, obstacles, k_neighbors, agents, simulators, delay_ratios,
                   delay_intervals, timeout):
    async def init_case(map_seed, agent_seed, obstacle, k_neighbor, agent, simulator, delay_ratio, delay_interval,
                        naive_feasibility, naive_cycle, only_cycle):
        return await run("random", feasibility_type=False, iteration=iteration,
                         map_seed=map_seed, agent_seed=agent_seed, obstacles=obstacle, agents=agent,
                         k_neighbor=k_neighbor, simulator=simulator, delay_type="agent",
                         delay_start=0, delay_ratio=delay_ratio, delay_interval=delay_interval,
                         naive_feasibility=naive_feasibility, naive_cycle=naive_cycle, only_cycle=only_cycle,
                         timeout=timeout)

    test_file = result_dir / "tests.csv"
    df = pd.read_csv(test_file, header=None)
    df.columns = ["map_seed", "agent_seed", "obstacle", "k_neighbor", "agent"]

    completed_file = result_dir / "completed.csv"
    if completed_file.exists():
        with completed_file.open("r") as f:
            for line in f.readlines():
                completed.add(line.strip())

    tasks = []
    for _map_seed in range(map_seeds):
        for _obstacle in obstacles:
            for _agent in agents:
                for _k_neighbor in k_neighbors:
                    df_temp = df[(df["map_seed"] == _map_seed) & (df["obstacle"] == _obstacle) & (
                            df["k_neighbor"] == _k_neighbor) & (df["agent"] == _agent)]
                    df_temp.sort_values(by="agent_seed")
                    if len(df_temp) < agent_seeds:
                        print("warning: %d %d %d %d no enough seeds" % (_map_seed, _obstacle, _k_neighbor, _agent))
                    for i in range(min(len(df_temp), agent_seeds)):
                        _agent_seed = df_temp["agent_seed"].iloc[i]
                        for _delay_ratio in delay_ratios:
                            for _delay_interval in delay_intervals:
                                for _simulator in simulators:
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


@click.command()
@click.option("--map-seeds", type=int, default=10)
@click.option("--agent-seeds", type=int, default=10)
@click.option("--iteration", type=int, default=10)
@click.option("--obstacles", type=str, default="90,270", callback=validate_list(int))
@click.option("--agents", type=str, default="10,20,30", callback=validate_list(int))
@click.option("--simulators", type=str, default="online,default,replan,pibt", callback=validate_list(str))
@click.option("--k-neighbors", type=str, default="2", callback=validate_list(int))
# @click.option("--delay-types", type=str, default="agent", callback=validate_list(str))
@click.option("--delay-ratios", type=str, default="0.1,0.2,0.3", callback=validate_list(float))
@click.option("--delay-intervals", type=str, default="1,10,20", callback=validate_list(int))
@click.option("-t", "--timeout", type=int, default=600)
@click.option("--init-tests", type=bool, default=False, is_flag=True)
@asyncio_wrapper
async def main(map_seeds, agent_seeds, iteration, obstacles, agents, k_neighbors,
               simulators, delay_ratios, delay_intervals, timeout, init_tests):
    os.chdir(result_dir)
    if init_tests:
        await do_init_tests(map_seeds, agent_seeds, obstacles, k_neighbors, agents, timeout)
    else:
        await do_tests(map_seeds, agent_seeds, iteration, obstacles, k_neighbors, agents, simulators, delay_ratios,
                       delay_intervals, timeout)


if __name__ == '__main__':
    main()
