# Multi-Agent Path Execution with Uncertainty

## Requirements

The program is written in C++17, built with CMake and boost library.

You can use the following command to install them on the **latest** version of Debian/Ubuntu.

```bash
$ sudo apt install g++ cmake libboost-all-dev
```

The script used to run the experiments and parse the results are written in Python 3.

The required packages can be installed by

```bash
python3 -m pip install pandas numpy matplotlib scipy 
```

## Build

```
mkdir cmake-build-relwithdebinfo
cd cmake-build-relwithdebinfo
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j
```

This will output a file `MAPF_DP` in the `cmake-build-relwithdebinfo` folder. You can change `RelWithDebInfo` to `Debug` and make a new directory for a debugging build.

## Usage

```asm
USAGE: MAPF_DP [OPTIONS]

OPTIONS:

-a, --agents ARG        Agents Number

-d, --debug             Debug Mode

-h, --help              Display this Message.

-i, --iteration ARG     Iteration Number

-m, --map ARG           Map type (random / warehouse)

-o, --output ARG        Output Filename

-p, --delay-start ARG   Iteration Number

-w, --window ARG        Window Size (0 means no limit, deprecated)

--agent-seed ARG        Agent Seed

--all                   All Constraints

--delay ARG             Delay type (agent / node / edge)

--delay-interval ARG    Delay Interval

--delay-ratio ARG       Delay Ratio

--dp                    Use delay probability (in solver)

--feasibility-type      Classify feasibility types

--map-seed ARG          Map Seed

--max ARG               Min Delay Probability (in solver)

--min ARG               Min Delay Probability (in solver)

--naive-cycle           Use naive cycle check

--naive-feasibility     Use naive feasibility check

--objective ARG         Objective type (maximum / average)

--obstacles ARG         Obstacles

--only-cycle            Use only cycle check

--simulation-seed ARG   Simulation Seed

--simulator ARG         Simulator type (default / online)

EXAMPLES:

./MAPF_DP --map random --map-seed 0 --agent-seed 0 --simulation-seed 0 --agents 20 --iteration 10 --simulator online --d
elay-start 1 --delay-ratio 0.3 --delay-interval 10 --obstacles 270 --delay agent

```

## Experiments

Note that the executable `cmake-build-relwithdebinfo/MAPF_DP` must be compiled first before running the experiments.

### Discrete

```bash
# run the tests
$ python3 experiments/test.py --timing discrete
# parse the results
$ python3 experiments/parse.py --timing discrete
# parse the results of the exhaustive vs. heuristic feasibility 
$ python3 experiments/parse.py --timing discrete --category -o feasibility_category   
# plot the figures
$ python3 experiments/plot.py --timing discrete
```

### Continuous

```bash
# run the tests
$ python3 experiments/test.py -j 10 --init-tests --map-seeds 10 --agent-seeds 10 --obstacles 90,180 --agents 20 --k-neighbors 2,3,4
$ python3 experiments/test.py -j 10 --map-seeds 10 --agent-seeds 10 --obstacles 90,180 --agents 20 --k-neighbors 2,3,4 --simulators snapshot_end,online,online_remove_redundant --delay-ratios 0.1 --delay-intervals 0
# parse the results
$ python3 experiments/parse.py --timing continuous --delay-intervals 0,1,10,20 --obstacles 90,180 --agents 20 --simulators online,snapshot_end,online_remove_redundant --k-neighbors 2,3,4 
# plot the figures
$ python3 experiments/plot.py --timing continuous --delay-intervals 0,1,10,20 --obstacles 90,180 --agents 20 --k-neighbors 2,3,4
```

## Algorithms

### Discrete

+ Algorithm 1: the function `feasibilityCheckHelper` in `src/simulator/DiscreteOnlineSimulator.cpp`
+ Algorithm 2: the framework is in the function `simulate` in `src/simulator/DiscreteOnlineSimulator.cpp`, the detailed implementation of line 12-23 is in the function `heuristicCycleCheck` in the same file.

### Continuous

+ `src/simulator/ContinuousOnlineSimulator.cpp`
+ `src/dependency_graph%2FNodeEdgeDependencyGraph.cpp`