#include <iostream>
#include <chrono>
#include <boost/filesystem.hpp>
#include <boost/dll.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include "Graph.h"
#include "solver/CBSSolver.h"
#include "solver/EECBSSolver.h"
#include "solver/CCBSSolver.h"
#include "solver/IndividualAStarSolver.h"
#include "simulator/ContinuousDefaultSimulator.h"
#include "simulator/ContinuousOnlineSimulator.h"
#include "simulator/DiscreteDefaultSimulator.h"
#include "simulator/DiscreteOnlineSimulator.h"
#include "simulator/DiscretePIBTSimulator.h"
#include "utils/ezOptionParser.hpp"


std::string double_to_string(double data) {
    auto result = std::to_string(data);
    auto pos = result.length() - 1;
    while (pos > 0 && result[pos] == '0') --pos;
    if (result[pos] == '.') --pos;
    return result.substr(0, pos + 1);
}

int main(int argc, const char *argv[]) {
    ez::ezOptionParser optionParser;

    optionParser.overview = "Multi Agent Path Finding with Delay Probability";
    optionParser.syntax = "./MAPF_DP [OPTIONS]";
    optionParser.example = "./MAPF_DP --map random --map-seed 0 --agent-seed 0 --simulation-seed 0 --agents 20 --iteration 10 --simulator online --delay-start 1 --delay-ratio 0.3 --delay-interval 10 --obstacles 270 --delay agent\n";
    optionParser.footer = "";

    optionParser.add("", false, 0, 0, "Display this Message.", "-h", "--help");
    optionParser.add("", false, 0, 0, "Debug Mode", "-d", "--debug");
    optionParser.add("", false, 0, 0, "All Constraints", "--all");
    optionParser.add("", false, 0, 0, "Use delay probability (in solver)", "--dp");
    optionParser.add("", false, 0, 0, "Use naive feasibility check", "--naive-feasibility");
    optionParser.add("", false, 0, 0, "Use naive cycle check", "--naive-cycle");
    optionParser.add("", false, 0, 0, "Use only cycle check", "--only-cycle");
    optionParser.add("", false, 0, 0, "Classify feasibility types", "--feasibility-type");
    optionParser.add("", false, 0, 0, "Use prioritized replan", "--prioritized-replan");
    optionParser.add("", false, 0, 0, "Use prioritized replan with optimization ", "--prioritized-opt");
    optionParser.add("", false, 0, 0, "Don't use cache for map generator and solver", "--no-cache");
    optionParser.add("random", false, 1, 0, "Map type (random / warehouse)", "-m", "--map");
    optionParser.add("", false, 1, 0, "Map file", "--map-file");
    optionParser.add("maximum", false, 1, 0, "Objective type (maximum / average)", "--objective");
    optionParser.add("default", false, 1, 0, "Simulator type (default / online / replan / pibt)", "--simulator");
    optionParser.add("continuous", false, 1, 0, "Timing type (discrete / continuous)", "--timing");
    optionParser.add("", false, 1, 0, "Statistics Output Filename", "-o", "--output");
    optionParser.add("", false, 1, 0, "Simulator Output Filename", "-o", "--simulator-output");
    optionParser.add("", false, 1, 0, "Time Output Filename", "-o", "--time-output");
    optionParser.add("edge", false, 1, 0, "Delay type (agent / node / edge)", "--delay");
    optionParser.add("eecbs", false, 1, 0, "Solver (default / separate / eecbs / ccbs", "--solver");
    optionParser.add("", false, 1, 0, "Solver binary (for CCBS / EECBS)", "--solver-binary");

    auto validWindowSize = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Window Size (0 means no limit, deprecated)", "-w", "--window", validWindowSize);

    auto validObstacles = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("90", false, 1, 0, "Obstacles", "--obstacles", validObstacles);

    auto validMapSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Map Seed", "--map-seed", validMapSeed);

    auto validAgentSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Agent Seed", "--agent-seed", validAgentSeed);

    auto validSimulationSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Simulation Seed", "--simulation-seed", validSimulationSeed);

    auto validAgents = new ez::ezOptionValidator("u4", "ge", "1");
    optionParser.add("10", false, 1, 0, "Agents Number", "-a", "--agents", validAgents);

    auto validIteration = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("10", false, 1, 0, "Iteration Number", "-i", "--iteration", validIteration);

    auto validMin = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0", false, 1, 0, "Min Delay Probability (in solver)", "--min", validMin);
    auto validMax = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0.5", false, 1, 0, "Min Delay Probability (in solver)", "--max", validMax);

    auto validDelayRatio = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0.2", false, 1, 0, "Delay Ratio", "--delay-ratio", validDelayRatio);

    auto validDelayInterval = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("1", false, 1, 0, "Delay Interval", "--delay-interval", validDelayInterval);

//    auto validDelayStart = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Delay Start", "-p", "--delay-start");

    auto validKNeighbor = new ez::ezOptionValidator("u4", "ge", "2");
    optionParser.add("2", false, 1, 0, "Max Edge Length", "--k-neighbor", validKNeighbor);

    auto validSuboptimality = new ez::ezOptionValidator("d", "ge", "1");
    optionParser.add("1", false, 1, 0, "Suboptimality of CBS", "--suboptimality", validSuboptimality);


    optionParser.parse(argc, argv);
    if (optionParser.isSet("-h")) {
        std::string usage;
        optionParser.getUsage(usage, 80, ez::ezOptionParser::ALIGN);
        std::cout << usage;
        return 1;
    }

    std::string mapType, objective, simulatorType, timingType, outputFileName, simulatorOutputFileName, timeOutputFileName, delayType, solverType, solverBinaryFile, mapFile;
    unsigned long window, mapSeed, agentSeed, simulationSeed, agentNum, iteration, obstacles, kNeighbor;
    long delayStart, delayInterval;
    double minDP, maxDP, delayRatio, suboptimality;
    bool debug, allConstraint, useDP, naiveFeasibilityCheck, naiveCycleCheck, onlyCycleCheck, feasibilityType, prioritizedReplan, prioritizedOpt, noCache;
    optionParser.get("--map")->getString(mapType);
    optionParser.get("--map-file")->getString(mapFile);
    optionParser.get("--objective")->getString(objective);
    optionParser.get("--simulator")->getString(simulatorType);
    optionParser.get("--timing")->getString(timingType);
    optionParser.get("--output")->getString(outputFileName);
    optionParser.get("--simulator-output")->getString(simulatorOutputFileName);
    optionParser.get("--time-output")->getString(timeOutputFileName);
    optionParser.get("--solver")->getString(solverType);
    optionParser.get("--solver-binary")->getString(solverBinaryFile);
    optionParser.get("--window")->getULong(window);
    optionParser.get("--obstacles")->getULong(obstacles);
    optionParser.get("--map-seed")->getULong(mapSeed);
    optionParser.get("--agent-seed")->getULong(agentSeed);
    optionParser.get("--simulation-seed")->getULong(simulationSeed);
    optionParser.get("--agents")->getULong(agentNum);
    optionParser.get("--iteration")->getULong(iteration);
    optionParser.get("--min")->getDouble(minDP);
    optionParser.get("--max")->getDouble(maxDP);
    optionParser.get("--delay")->getString(delayType);
    optionParser.get("--delay-ratio")->getDouble(delayRatio);
    optionParser.get("--delay-interval")->getLong(delayInterval);
    optionParser.get("--delay-start")->getLong(delayStart);
    optionParser.get("--k-neighbor")->getULong(kNeighbor);
    optionParser.get("--suboptimality")->getDouble(suboptimality);
    debug = optionParser.isSet("--debug");
    allConstraint = optionParser.isSet("--all");
    useDP = optionParser.isSet("--dp");
    naiveFeasibilityCheck = optionParser.isSet("--naive-feasibility");
    naiveCycleCheck = optionParser.isSet("--naive-cycle");
    onlyCycleCheck = optionParser.isSet("--only-cycle");
    feasibilityType = optionParser.isSet("--feasibility-type");
    prioritizedReplan = optionParser.isSet("--prioritized-replan");
    prioritizedOpt = optionParser.isSet("--prioritized-opt");
    noCache = optionParser.isSet("--no-cache");

    if (window == 0) {
        window = INT_MAX;
    }
    if (delayInterval < 0) {
        delayInterval = INT_MAX;
    }
    if (delayStart < 0) {
        delayStart = INT_MAX;
    }
    if (solverBinaryFile.empty()) {
        auto solverBinaryPath = boost::dll::program_location().parent_path();
        if (solverType == "ccbs") {
            solverBinaryPath = solverBinaryPath / "Continuous-CBS" / "CCBS";
        } else if (solverType == "eecbs") {
            solverBinaryPath = solverBinaryPath / "EECBS" / "eecbs";
        }
        if (!solverBinaryPath.empty()) {
            solverBinaryFile = solverBinaryPath.string();
        }
    } else {
        auto solverBinaryPath = boost::filesystem::path(solverBinaryFile);
        solverBinaryFile = boost::filesystem::canonical(solverBinaryPath).string();
    }
    std::cout << solverBinaryFile << std::endl;
//    std::cout << "window: " << window << std::endl;

    std::ofstream fout;
    auto buf = std::make_unique<char[]>(4096);
    if (!outputFileName.empty()) {
        fout.open(outputFileName, std::ios_base::app);
        fout.rdbuf()->pubsetbuf(buf.get(), 4096);
        std::cerr << "redirect output to " << outputFileName << std::endl;
    }
    std::ostream &out = fout.is_open() ? fout : std::cout;

    std::string lockFileName = outputFileName + ".lock";
    std::ofstream lockFile(lockFileName, std::ios_base::app);
    lockFile.close();


    Graph graph;
    unsigned int height = 30;
    unsigned int width = 30;
//    unsigned int obstacles = height * width * obstacleRate;

    unsigned int deliveryWidth = 10;
    unsigned int deliveryX = 7;
    unsigned int deliveryY = 4;
    unsigned int maxX = 3 * deliveryX + 1;
    unsigned int maxY = deliveryY * (deliveryWidth + 1) + 13;

//    unsigned int agents = 30;
    graph.initDelayProbability(minDP, maxDP);
    graph.debug = debug;
//    graph.noCache = noCache;

    std::string filename;
    if (mapType == "random") {
        filename = timingType + "-random-" + std::to_string(height) + "-" + std::to_string(width) + "-" + std::to_string(obstacles) +
                   "-" + std::to_string(mapSeed) + "-" + std::to_string(kNeighbor);
        graph.generateRandomGraph(height, width, obstacles, filename, mapSeed, kNeighbor);
    } else if (mapType == "warehouse") {
        filename = "warehouse-" + std::to_string(maxX) + "-" + std::to_string(maxY);
        graph.generateWareHouse(deliveryWidth, maxX, maxY, filename, mapSeed);
    } else if (mapType == "hardcoded") {
        filename = "hardcoded";
        graph.generateHardCodedGraph(filename, mapSeed);
        agentNum = 3;
        agentSeed = 1;
    } else if (mapType == "dot") {
        filename = mapFile;
        graph.generateDOTGraph(filename);
    } else if (mapType == "graphml") {
        filename = mapFile;
        graph.generateGraphMLGraph(filename);
    }
    if (useDP) {
        graph.generateDelayProbability(mapSeed, minDP, maxDP);
    }
    graph.calculateAllPairShortestPath(filename, useDP);

    std::vector<Agent> agents;
    if (mapType == "random") {
        agents = graph.generateRandomAgents(agentNum, agentSeed);
    } else if (mapType == "warehouse") {
        agents = graph.generateWarehouseAgents(agentNum, agentSeed, true);
    } else if (mapType == "hardcoded") {
        agents = graph.generateHardCodedAgents(agentNum);
    } else if (mapType == "dot") {
        agents = graph.generateRandomAgents(agentNum, agentSeed);
    }  else if (mapType == "graphml") {
        agents = graph.generateRandomAgents(agentNum, agentSeed);
    }

    MakeSpanType makeSpanType = MakeSpanType::UNKNOWN;
    if (objective == "maximum") {
        makeSpanType = MakeSpanType::MAXIMUM;
    } else if (objective == "average") {
        makeSpanType = MakeSpanType::AVERAGE;
    } else {
        assert(0);
    }

    std::shared_ptr<Solver> solver;
    if (solverType == "default") {
        solver = std::shared_ptr<Solver>(new CBSSolver(graph, agents, makeSpanType, window));
    } else if (solverType == "eecbs") {
        if (solverBinaryFile.empty()) exit(-1);
        solver = std::shared_ptr<Solver>(new EECBSSolver(graph, agents, makeSpanType, solverBinaryFile, suboptimality));
    } else if (solverType == "ccbs") {
        if (solverBinaryFile.empty()) exit(-1);
        solver = std::shared_ptr<Solver>(new CCBSSolver(graph, agents, makeSpanType, solverBinaryFile));
    } else if (solverType == "individual") {
        solver = std::shared_ptr<Solver>(new IndividualAStarSolver(graph, agents, makeSpanType));
    } else {
        assert(0);
    }

//    CBSSolver solver(graph, agents);
    solver->debug = debug;
    solver->useDP = useDP;
    solver->allConstraint = allConstraint;
    solver->init();
    bool result;
    if (noCache) {
        result = solver->solve();
    } else {
        result = solver->solveWithCache(filename, agentSeed);
    }

    if (!result) {
        exit(-1);
    }

//    if (!timeOutputFileName.empty()) {
//        // delete old file
//        std::ofstream tempFile(timeOutputFileName);
//        tempFile.close();
//    }

//    double approx = solver->approxAverageMakeSpan(*solver->solution);
//    std::shared_ptr<ContinuousOnlineSimulator> onlineSimulator;
    std::shared_ptr<Simulator> simulator;

    unsigned int finished = 0;

    for (unsigned int i = simulationSeed; finished < iteration; i++) {
        if (useDP) {
            graph.generateDelayProbability(i, minDP, maxDP);
        }

        if (simulatorType == "default" || simulatorType == "replan") {
            if (timingType == "continuous") {
                simulator = std::make_unique<ContinuousDefaultSimulator>(graph, agents, i);
            } else {
                simulator = std::make_unique<DiscreteDefaultSimulator>(graph, agents, i);
            }
            if (simulatorType == "replan") {
                auto defaultSimulator = std::dynamic_pointer_cast<DefaultSimulator>(simulator);
                defaultSimulator->replanMode = true;
                defaultSimulator->prioritizedReplan = prioritizedReplan;
                defaultSimulator->prioritizedOpt = prioritizedOpt;
                if (i != 0) {
                    solver->init();
                    if (!solver->solveWithCache(filename, agentSeed)) {
                        exit(-1);
                    }
                }
            }
        } else if (simulatorType == "online") {
            if (timingType == "continuous") {
                simulator = std::make_unique<ContinuousOnlineSimulator>(graph, agents, i);
            } else {
                simulator = std::make_unique<DiscreteOnlineSimulator>(graph, agents, i);
            }
            auto onlineSimulator = std::dynamic_pointer_cast<OnlineSimulator>(simulator);
            onlineSimulator->isHeuristicFeasibilityCheck = !naiveFeasibilityCheck;
            onlineSimulator->isHeuristicCycleCheck = !naiveCycleCheck;
            onlineSimulator->isOnlyCycleCheck = onlyCycleCheck;
            onlineSimulator->isFeasibilityType = feasibilityType;
        } else if (simulatorType == "pibt") {
            if (timingType == "continuous") {
                assert(0);
            } else {
                simulator = std::make_unique<DiscretePIBTSimulator>(graph, agents, i);
            }
        } else {
            assert(0);
        }
        simulator->delayRatio = delayRatio;
        simulator->delayType = delayType;
        simulator->setSolver(solver);
        simulator->debug = debug;
        simulator->outputFileName = simulatorOutputFileName;
        simulator->timeOutputFileName = timeOutputFileName;

/*        if (mapType == "hardcoded") {
            CBSNodePtr solution = std::make_shared<CBSNode>();
            std::shared_ptr<AgentPlan> a0 = std::make_shared<AgentPlan>();
            std::shared_ptr<AgentPlan> a1 = std::make_shared<AgentPlan>();
            std::shared_ptr<AgentPlan> a2 = std::make_shared<AgentPlan>();
            solution->plans.push_back(a0);
            solution->plans.push_back(a1);
            solution->plans.push_back(a2);
            (*a0).add(6).add(7).add(8).add(9).add(10).add(11).add(12).add(13);
            (*a1).add(0).add(2).add(4).add(4).add(4).add(10).add(9).add(14).add(15).add(16);
            (*a2).add(1).add(3).add(5).add(5).add(5).add(5).add(5).add(12);
            simulator->setSolution(solution);

        } else {

        }*/

        double currentTimestep = 1;
        const int maxTimestep = 300;

#ifdef DEBUG_CYCLE
        simulator->debug = true;
#endif
        simulator->setAgents(agents);
        currentTimestep = 0;
        auto start = std::chrono::steady_clock::now();
        auto count = simulator->simulate(currentTimestep, currentTimestep + maxTimestep, delayStart, delayInterval);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;

        if (count == agentNum) {
            boost::interprocess::file_lock flock(lockFileName.c_str());
            flock.lock();
            out << mapSeed << "," << agentSeed << "," << i << ",";
            if (delayInterval == INT_MAX) {
                out << count << "," << finished;
            } else {
                out << simulator->averageMakeSpan(MakeSpanType::MAXIMUM) << ","
                    << simulator->averageMakeSpan(MakeSpanType::AVERAGE);
            }
            out << "," << elapsed_seconds.count() << ",";
            simulator->print(out);
            out << std::endl;
            simulator->printExecutionTime(mapSeed, agentSeed, i);
            finished++;
            flock.unlock();
        } else {
            std::cerr << count << " " << agentNum << std::endl;
            finished++;
//            exit(-1);
        }
    }


    if (fout.is_open()) {
        fout.close();
    }

    return 0;
}
