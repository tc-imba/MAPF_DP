import asyncio
import os
import subprocess
import functools

project_root = os.path.dirname(os.path.dirname(__file__))
program = os.path.join(project_root, "cmake-build-relwithdebinfo", "MAPF_DP.exe")
data_root = os.path.join(project_root, "maps")
result_dir = os.path.join(project_root, "result")
os.makedirs(data_root, exist_ok=True)
os.makedirs(result_dir, exist_ok=True)

workers = 1
count = 0

TIMEOUT = 36000
EXPERIMENT_JOBS = 36


async def run(map_type, objective="maximum", seed=0, agents=35, iteration=1000, min_dp=0.25, max_dp=0.75,
              simulator="online", pause=10, delay_ratio=0.2, delay_interval=1):
    # base_filename = "%d-%d-%d-%d-%d" % (size[0], size[1], agent, task_per_agent, seed)
    # task_filename = "task/well-formed-%s.task" % base_filename
    # phi_output = phi >= 0 and str(phi) or 'n' + str(-phi)
    output_filename = "%s-agents-%d-%d-%s-%s.csv" % (simulator, agents, pause, delay_ratio, delay_interval)
    args = [
        # program,
        "--map", map_type,
        "--objective", objective,
        "--seed", str(seed),
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
    print(' '.join(args))
    global workers, count
    while workers <= 0:
        await asyncio.sleep(1)

    workers -= 1
    p = None
    try:
        p = await asyncio.create_subprocess_exec(program, *args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        await asyncio.wait_for(p.communicate(), timeout=TIMEOUT)
    except asyncio.TimeoutError:
        print('timeout!')
    except Exception as e:
        print(e)
    try:
        p.kill()
    except:
        pass
    workers += 1
    count += 1
    print('%s (%d/%d)' % (output_filename, count, EXPERIMENT_JOBS))


async def main():
    os.chdir(result_dir)
    for agents in [10, 20, 30]:
        for pause in [1, 5, 10]:
            for delay_ratio in [0.2, 0.4]:
                for simulator in ["default", "online"]:
                    await run("random", min_dp=0.5, max_dp=0.9,
                              agents=agents, simulator=simulator,
                              pause=pause, delay_ratio=delay_ratio, delay_interval=0)
    for agents in [10, 20, 30]:
        for delay_ratio in [0.2, 0.4]:
            for delay_interval in [1, 3, 5, 10]:
                for simulator in ["default", "online"]:
                    await run("random", min_dp=0.5, max_dp=0.9,
                              agents=agents, simulator=simulator,
                              pause=0, delay_ratio=delay_ratio,
                              delay_interval=delay_interval)

    # for window in [0, 10, 20, 30]:
    #     await run("random", window=window)


if __name__ == '__main__':
    asyncio.run(main())
