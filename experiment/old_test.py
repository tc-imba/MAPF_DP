import asyncio
import os
import subprocess
import platform
from pathlib import Path
import multiprocessing

project_root = os.path.dirname(os.path.dirname(__file__))
program = os.path.join(project_root, "cmake-build-relwithdebinfo", "MAPF_DP")
if platform.system() == "Windows":
    program += ".exe"
data_root = os.path.join(project_root, "maps")
result_dir = os.path.join(project_root, "result")
os.makedirs(data_root, exist_ok=True)
os.makedirs(result_dir, exist_ok=True)

workers = multiprocessing.cpu_count()
count = 0

TIMEOUT = 600
MAP_SEEDS = 10
AGENT_SEEDS = 10
ITERATIONS = 10
OBSTACLES = [90, 270]
# OBSTACLES = [90]
# AGENTS = [10, 20]
AGENTS = [10, 20, 30]
EDGE_DELAY_RATIOS = [0.01, 0.05]
AGENT_DELAY_RATIOS = [0.1, 0.2, 0.3]

# DELAY_INTERVALS = range(1, 10)
DELAY_INTERVALS = [1, 10, 20]  # test 2
# DELAY_INTERVALS = [20, 30]
# PAUSES = range(1, 10)
DELAY_STARTS = [1, 5, 10]  # test 1, not used
DELAY_TYPES = ["agent"]
# SIMULATORS = ["online", "default", "replan"]
SIMULATORS = ["prioritized"]
NAIVE_SETTINGS = [
    (False, False, False),     # online/default,cycle
    (False, True, False),      # feasibility,cycle
    (False, True, True),       # cycle
    # (True, False, False),
    (True, True, False),       # feasibility
    # (True, True, True),
]
# FEASIBILITY_TYPES = [False]
FEASIBILITY_TYPES = [True, False]
EXPERIMENT_JOBS = 0
# EXPERIMENT_JOBS = MAP_SEEDS * AGENT_SEEDS * len(OBSTACLES) * len(AGENTS) * len(AGENT_DELAY_RATIOS) * \
#                   len(DELAY_TYPES) * len(DELAY_INTERVALS) * len(SIMULATORS) * len(NAIVE_SETTINGS)

result_files = set()
failed_settings = set()


async def run(map_type, objective="maximum", map_seed=0, agent_seed=0, agents=35, iteration=ITERATIONS, min_dp=0.25,
              max_dp=0.75, obstacles=90, simulator="online",
              delay_type="edge", delay_ratio=0.2, delay_start=0, delay_interval=0,
              naive_feasibility=False, naive_cycle=False, only_cycle=False, feasibility_type=False):
    global workers, count, EXPERIMENT_JOBS

    # base_filename = "%d-%d-%d-%d-%d" % (size[0], size[1], agent, task_per_agent, seed)
    # task_filename = "task/well-formed-%s.task" % base_filename
    # phi_output = phi >= 0 and str(phi) or 'n' + str(-phi)
    output_filename = "%s-%d-%d-%s-%s-%d-%d-%s-%s.csv" % (
        simulator, obstacles, agents, delay_type, delay_ratio, delay_start, delay_interval,
        naive_feasibility and "n" or "h",
        only_cycle and "o" or (naive_cycle and "n" or "h"),
    )
    cbs_prefix = "random-30-30-%d-%d-%d-%d-eecbs" % (obstacles, map_seed, agents, agent_seed)
    cbs_file = Path(result_dir) / (cbs_prefix + ".cbs")
    cbs_failed_file = Path(result_dir) / (cbs_prefix + ".failed")
    settings_name = "%d-%d-%d" % (map_seed, agent_seed, agents)
    EXPERIMENT_JOBS += 1

    if cbs_failed_file.exists():
        count += 1
        print('%s skipped (%d/%d)' % (output_filename, count, EXPERIMENT_JOBS))
        return

        # if settings_name in failed_settings:
    #     return

    if output_filename not in result_files:
        result_files.add(output_filename)
        file: Path = Path(result_dir) / output_filename
        file.unlink(missing_ok=True)

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
        "--min", str(min_dp),
        "--max", str(max_dp),
        "--obstacles", str(obstacles),
        "--simulator", simulator,
        "--delay", delay_type,
        "--delay-ratio", str(delay_ratio),
        "--delay-start", str(delay_start),
        "--delay-interval", str(delay_interval),
        "--output", os.path.join(result_dir, output_filename),
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
    # print(' '.join(args))
    while workers <= 0:
        await asyncio.sleep(1)

    workers -= 1
    p = None
    try:
        p = await asyncio.create_subprocess_exec(program, *args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        await asyncio.wait_for(p.communicate(), timeout=TIMEOUT)
    except asyncio.TimeoutError:
        # failed_settings.add(settings_name)
        print('timeout!')
    except Exception as e:
        print(e)
    try:
        p.kill()
    except:
        pass

    if not cbs_file.exists():
        cbs_failed_file.touch(exist_ok=True)

    workers += 1
    count += 1
    # print(program)
    # print(' '.join(args))
    print('%s completed (%d/%d)' % (output_filename, count, EXPERIMENT_JOBS))


def get_delay_ratios():
    for delay_type in DELAY_TYPES:
        if delay_type == "edge":
            yield delay_type, EDGE_DELAY_RATIOS
        elif delay_type == "agent":
            yield delay_type, AGENT_DELAY_RATIOS
        else:
            assert False


async def run_test_1(map_seed, agent_seed, agents, obstacles, feasibility_type):
    for delay_type, delay_ratios in get_delay_ratios():
        for delay_start in DELAY_STARTS:
            for delay_ratio in delay_ratios:
                for simulator in SIMULATORS:
                    if simulator != "online" and feasibility_type:
                        continue
                    elif simulator != "online" or feasibility_type:
                        naive_settings = [(False, False, False)]
                    else:
                        naive_settings = NAIVE_SETTINGS
                    for (naive_feasibility, naive_cycle, only_cycle) in naive_settings:
                        await run("random", min_dp=0.5, max_dp=0.9, feasibility_type=feasibility_type,
                                  map_seed=map_seed, agent_seed=agent_seed, obstacles=obstacles,
                                  agents=agents, simulator=simulator, delay_type=delay_type,
                                  delay_start=delay_start, delay_ratio=delay_ratio, delay_interval=0,
                                  naive_feasibility=naive_feasibility, naive_cycle=naive_cycle, only_cycle=only_cycle)


async def run_test_2(map_seed, agent_seed, agents, obstacles, feasibility_type):
    for delay_type, delay_ratios in get_delay_ratios():
        for delay_ratio in delay_ratios:
            for delay_interval in DELAY_INTERVALS:
                for simulator in SIMULATORS:
                    if simulator != "online" and feasibility_type:
                        continue
                    elif simulator != "online" or feasibility_type:
                        naive_settings = [(False, False, False)]
                    else:
                        naive_settings = NAIVE_SETTINGS
                    for (naive_feasibility, naive_cycle, only_cycle) in naive_settings:
                        await run("random", min_dp=0.5, max_dp=0.9, feasibility_type=feasibility_type,
                                  map_seed=map_seed, agent_seed=agent_seed, obstacles=obstacles,
                                  agents=agents, simulator=simulator, delay_type=delay_type,
                                  delay_start=1, delay_ratio=delay_ratio, delay_interval=delay_interval,
                                  naive_feasibility=naive_feasibility, naive_cycle=naive_cycle, only_cycle=only_cycle)


async def main():
    os.chdir(result_dir)
    # for file in Path(result_dir).iterdir():
    #     print(file)

    tasks = []
    for map_seed in range(MAP_SEEDS):
        for agent_seed in range(AGENT_SEEDS):
            for obstacles in OBSTACLES:
                for agents in AGENTS:
                    if obstacles > 270 and agents > 10:
                        continue
                    for feasibility_type in FEASIBILITY_TYPES:
                        # tasks.append(
                        #     run_test_1(
                        #         map_seed=map_seed, agent_seed=agent_seed, agents=agents, obstacles=obstacles,
                        #         feasibility_type=feasibility_type
                        #     )
                        # )
                        tasks.append(
                            run_test_2(
                                map_seed=map_seed, agent_seed=agent_seed, agents=agents, obstacles=obstacles,
                                feasibility_type=feasibility_type
                            )
                        )

    await asyncio.gather(*tasks)

    # for agents in [10, 20, 30]:
    #     for delay_ratio in [0.2, 0.4]:
    #         for delay_interval in [1, 3, 5, 10]:
    #             for simulator in ["default", "online"]:
    #                 await run("random", min_dp=0.5, max_dp=0.9,
    #                           agents=agents, simulator=simulator,
    #                           pause=0, delay_ratio=delay_ratio,
    #                           delay_interval=delay_interval)

    # for window in [0, 10, 20, 30]:
    #     await run("random", window=window)


if __name__ == '__main__':
    asyncio.run(main())
