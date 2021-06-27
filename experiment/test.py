import asyncio
import os
import subprocess
import functools

project_root = os.path.dirname(os.path.dirname(__file__))
program = os.path.join(project_root, "cmake-build-release", "MAPF_DP")
data_root = os.path.join(project_root, "maps")
result_dir = os.path.join(project_root, "result")
os.makedirs(data_root, exist_ok=True)
os.makedirs(result_dir, exist_ok=True)

workers = 1
count = 0

TIMEOUT = 36000
EXPERIMENT_JOBS = 10


async def run(map_type, objective="maximum", window=0, seed=0, agents=35, iteration=1000, min_dp=0.25, max_dp=0.75):
    # base_filename = "%d-%d-%d-%d-%d" % (size[0], size[1], agent, task_per_agent, seed)
    # task_filename = "task/well-formed-%s.task" % base_filename
    # phi_output = phi >= 0 and str(phi) or 'n' + str(-phi)
    output_filename = "%s-%s-window-%d-seed-%d-%s-%s.csv" % (map_type, objective, window, seed, min_dp, max_dp)
    args = [
        program,
        "--map", map_type,
        "--objective", objective,
        "--window", str(window),
        "--seed", str(seed),
        "--agents", str(agents),
        "--iteration", str(iteration),
        "--min", str(min_dp),
        "--max", str(max_dp),
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
        p = await asyncio.create_subprocess_exec(program, *args, stderr=subprocess.PIPE)
        await asyncio.wait_for(p.communicate(), timeout=TIMEOUT)
    except asyncio.TimeoutError:
        print('timeout!')
    except:
        pass
    try:
        p.kill()
    except:
        pass
    workers += 1
    count += 1
    print('%s (%d/%d)' % (output_filename, count, EXPERIMENT_JOBS))


async def main():
    os.chdir(data_root)
    for window in [0, 10, 5, 2]:
        await run("hardcoded", window=window)
    # for window in [0, 10, 20, 30]:
    #     await run("random", window=window)

if __name__ == '__main__':
    asyncio.run(main())
