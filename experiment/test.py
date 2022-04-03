import asyncio
import os
import subprocess
import functools
from pathlib import Path

project_root = os.path.dirname(os.path.dirname(__file__))
program = os.path.join(project_root, "cmake-build-relwithdebinfo", "MAPF_DP.exe")
data_root = os.path.join(project_root, "maps")
result_dir = os.path.join(project_root, "result")
os.makedirs(data_root, exist_ok=True)
os.makedirs(result_dir, exist_ok=True)

workers = 100
count = 0

TIMEOUT = 60
EXPERIMENT_JOBS = 100 * (36 + 48) * 2

result_files = set()
failed_settings = set()


async def run(map_type, objective="maximum", map_seed=0, agent_seed=0, agents=35, iteration=10, min_dp=0.25,
              max_dp=0.75, obstacles=90,
              simulator="online", pause=10, delay_ratio=0.2, delay_interval=1):
    # base_filename = "%d-%d-%d-%d-%d" % (size[0], size[1], agent, task_per_agent, seed)
    # task_filename = "task/well-formed-%s.task" % base_filename
    # phi_output = phi >= 0 and str(phi) or 'n' + str(-phi)
    output_filename = "%s-%d-%d-%d-%s-%s.csv" % (simulator, obstacles, agents, pause, delay_ratio, delay_interval)
    settings_name = "%d-%d-%d" % (map_seed, agent_seed, agents)

    if settings_name in failed_settings:
        return

    if output_filename not in result_files:
        result_files.add(output_filename)
        file: Path = Path(result_dir) / output_filename
        file.unlink(missing_ok=True)

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
        "--simulator", simulator,
        "--pause", str(pause),
        "--delay-ratio", str(delay_ratio),
        "--delay-interval", str(delay_interval),
        "--output", os.path.join(result_dir, output_filename),
    ]
    if map_type == "hardcoded":
        args.append("--all")
    # print(' '.join(args))
    global workers, count
    while workers <= 0:
        await asyncio.sleep(1)

    workers -= 1
    p = None
    try:
        p = await asyncio.create_subprocess_exec(program, *args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        await asyncio.wait_for(p.communicate(), timeout=TIMEOUT)
    except asyncio.TimeoutError:
        failed_settings.add(settings_name)
        print('timeout!')
    except Exception as e:
        print(e)
    try:
        p.kill()
    except:
        pass
    workers += 1
    count += 1
    # print(' '.join(args))
    print('%s (%d/%d)' % (output_filename, count, EXPERIMENT_JOBS))


async def run_test_1(map_seed, agent_seed, agents, obstacles):
    for pause in [1, 5, 10]:
        for delay_ratio in [0.2, 0.4]:
            for simulator in ["default", "online"]:
                await run("random", min_dp=0.5, max_dp=0.9,
                          map_seed=map_seed, agent_seed=agent_seed, obstacles=obstacles,
                          agents=agents, simulator=simulator,
                          pause=pause, delay_ratio=delay_ratio, delay_interval=0)


async def run_test_2(map_seed, agent_seed, agents, obstacles):
    for delay_ratio in [0.2, 0.4]:
        for delay_interval in [1, 3, 5, 10]:
            for simulator in ["default", "online"]:
                await run("random", min_dp=0.5, max_dp=0.9,
                          map_seed=map_seed, agent_seed=agent_seed, obstacles=obstacles,
                          agents=agents, simulator=simulator,
                          pause=0, delay_ratio=delay_ratio, delay_interval=delay_interval)


async def main():
    os.chdir(result_dir)
    for file in Path(result_dir).iterdir():
        print(file)

    tasks = []
    for map_seed in range(10):
        for agent_seed in range(10):
            for obstacles in [90, 180, 270]:
                for agents in [10]:
                    tasks.append(
                        run_test_1(map_seed=map_seed, agent_seed=agent_seed, agents=agents, obstacles=obstacles)
                    )
                    tasks.append(
                        run_test_2(map_seed=map_seed, agent_seed=agent_seed, agents=agents, obstacles=obstacles)
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
