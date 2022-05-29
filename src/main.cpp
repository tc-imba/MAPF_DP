#include <iostream>
#include <fstream>
#include <chrono>
#include "Graph.h"
#include "CBSSolver.h"
#include "DefaultSimulator.h"
#include "OnlineSimulator.h"
#include "ezOptionParser.hpp"

int main(int argc, const char *argv[]) {
    ez::ezOptionParser optionParser;

    optionParser.overview = "Multi Agent Path Finding with Delay Probability";
    optionParser.syntax = "./MAPF_DP [OPTIONS]";
//    optionParser.example = "./MAPF --flex -a 0 --phi 0 -b -s -m -db -re -d test-benchmark -t task/well-formed-21-35-10-2.task -o auto\n";
    optionParser.footer = "";

    optionParser.add("", false, 0, 0, "Display this Message.", "-h", "--help");
    optionParser.add("", false, 0, 0, "Debug Mode", "-d", "--debug");
    optionParser.add("", false, 0, 0, "All Constraints", "--all");
    optionParser.add("", false, 0, 0, "Use delay probability", "--dp");
    optionParser.add("", false, 0, 0, "Use naive feasibility check", "--naive-feasibility");
    optionParser.add("", false, 0, 0, "Use naive cycle check", "--naive-cycle");
    optionParser.add("random", false, 1, 0, "Map type (random / warehouse)", "-m", "--map");
    optionParser.add("maximum", false, 1, 0, "Objective type (maximum / average)", "--objective");
    optionParser.add("default", false, 1, 0, "Simulator type (default / online)", "--simulator");
    optionParser.add("", false, 1, 0, "Output Filename", "-o", "--output");

    auto validWindowSize = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Window Size (0 means no limit)", "-w", "--window", validWindowSize);

    auto validObstacles = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("90", false, 1, 0, "Obstacles", "--obstacles", validObstacles);

    auto validMapSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Simulation Seed", "--map-seed", validMapSeed);

    auto validAgentSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Simulation Seed", "--agent-seed", validAgentSeed);

    auto validAgents = new ez::ezOptionValidator("u4", "ge", "1");
    optionParser.add("10", false, 1, 0, "Agents Number", "-a", "--agents", validAgents);

    auto validIteration = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("10", false, 1, 0, "Iteration Number", "-i", "--iteration", validIteration);

    auto validPause = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Iteration Number", "-p", "--pause", validPause);

    auto validMin = new ez::ezOptionValidator("d", "ge", "0");
    auto validMax = new ez::ezOptionValidator("d", "ge", "0");
    auto validDelayRatio = new ez::ezOptionValidator("d", "ge", "0");
    auto validDelayInterval = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Min Delay Probability", "--min", validMin);
    optionParser.add("0.5", false, 1, 0, "Min Delay Probability", "--max", validMax);
    optionParser.add("0.2", false, 1, 0, "Delay Ratio", "--delay-ratio", validDelayRatio);
    optionParser.add("1", false, 1, 0, "Delay Interval", "--delay-interval", validDelayInterval);

    optionParser.parse(argc, argv);
    if (optionParser.isSet("-h")) {
        std::string usage;
        optionParser.getUsage(usage, 80, ez::ezOptionParser::ALIGN);
        std::cout << usage;
        return 1;
    }

    std::string mapType, objective, simulatorType, outputFileName;
    unsigned long window, mapSeed, agentSeed, agentNum, iteration, pause, delayInterval, obstacles;
    double minDP, maxDP, delayRatio;
    bool debug, allConstraint, useDP, naiveFeasibilityCheck, naiveCycleCheck;
    optionParser.get("--map")->getString(mapType);
    optionParser.get("--objective")->getString(objective);
    optionParser.get("--simulator")->getString(simulatorType);
    optionParser.get("--output")->getString(outputFileName);
    optionParser.get("--window")->getULong(window);
    optionParser.get("--obstacles")->getULong(obstacles);
    optionParser.get("--map-seed")->getULong(mapSeed);
    optionParser.get("--agent-seed")->getULong(agentSeed);
    optionParser.get("--agents")->getULong(agentNum);
    optionParser.get("--iteration")->getULong(iteration);
    optionParser.get("--pause")->getULong(pause);
    optionParser.get("--min")->getDouble(minDP);
    optionParser.get("--max")->getDouble(maxDP);
    optionParser.get("--delay-ratio")->getDouble(delayRatio);
    optionParser.get("--delay-interval")->getULong(delayInterval);
    debug = optionParser.isSet("--debug");
    allConstraint = optionParser.isSet("--all");
    useDP = optionParser.isSet("--dp");
    naiveFeasibilityCheck = optionParser.isSet("--naive-feasibility");
    naiveCycleCheck = optionParser.isSet("--naive-cycle");

    if (window == 0) {
        window = std::numeric_limits<unsigned int>::max() / 2;
    }
//    std::cout << "window: " << window << std::endl;

    std::ofstream fout;
    if (!outputFileName.empty()) {
        fout.open(outputFileName, std::ios_base::app);
        std::cerr << outputFileName << std::endl;
    }
    std::ostream &out = fout.is_open() ? fout : std::cout;


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


    std::string filename;
    if (mapType == "random") {
        filename = "random-" + std::to_string(height) + "-" + std::to_string(width) + "-" + std::to_string(obstacles) +
                   "-" + std::to_string(mapSeed);
        graph.generateRandomGraph(height, width, obstacles, filename, mapSeed);
    } else if (mapType == "warehouse") {
        filename = "warehouse-" + std::to_string(maxX) + "-" + std::to_string(maxY);
        graph.generateWareHouse(deliveryWidth, maxX, maxY, filename, mapSeed);
    } else if (mapType == "hardcoded") {
        filename = "hardcoded";
        graph.generateHardCodedGraph(filename, mapSeed);
        agentNum = 3;
        agentSeed = 1;
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
    }

    CBSSolver solver(graph, agents);
    solver.debug = debug;
    solver.useDP = useDP;
    solver.allConstraint = allConstraint;

    MakeSpanType makeSpanType = MakeSpanType::UNKNOWN;
    if (objective == "maximum") {
        makeSpanType = MakeSpanType::MAXIMUM;
    } else if (objective == "average") {
        makeSpanType = MakeSpanType::AVERAGE;
    } else {
        assert(0);
    }
    solver.init(0, makeSpanType);
    solver.initCBS(window);
//    while (solver.step()) {}
    solver.solveWithCache(filename, agentSeed);
    double approx = solver.approxAverageMakeSpan(*solver.solution);
    std::unique_ptr<Simulator> simulator;

    unsigned int finished = 0;

    for (unsigned int i = 0; finished < iteration; i++) {
        graph.generateDelayProbability(i, minDP, maxDP);

        if (simulatorType == "default") {
            simulator = std::make_unique<DefaultSimulator>(graph, agents, i);
        } else if (simulatorType == "online") {
            auto onlineSimulator = std::make_unique<OnlineSimulator>(graph, agents, i);
            onlineSimulator->isHeuristicFeasibilityCheck = !naiveFeasibilityCheck;
            onlineSimulator->isHeuristicCycleCheck = !naiveCycleCheck;
            simulator = std::move(onlineSimulator);
        } else {
            assert(0);
        }
        simulator->delayRatio = delayRatio;
        simulator->delayInterval = delayInterval;

        if (mapType == "hardcoded") {
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
            simulator->setSolution(solver.solution);
        }

        unsigned int currentTimestep = 1;
        auto start = std::chrono::steady_clock::now();
        int count = simulator->simulate(currentTimestep, currentTimestep + 1000);
        auto end = std::chrono::steady_clock::now();
        if (count == agentNum) {
            finished++;
            if (pause == 0) {
                std::chrono::duration<double> elapsed_seconds = end - start;
                out << simulator->averageMakeSpan(makeSpanType) << "," << approx << ","
                    << elapsed_seconds.count() << std::endl;
            } else {
                simulator->setAgents(agents);
                currentTimestep = 1;
                start = std::chrono::steady_clock::now();
                count = simulator->simulate(currentTimestep, currentTimestep + 1000, pause);
                end = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed_seconds = end - start;
                out << count << "," << i << "," << elapsed_seconds.count() << std::endl;
            }
        }


    }


/*    for (unsigned int i = 0; i < iteration; i++) {
        if (objective == "maximum") {
            solver.init(i, MakeSpanType::MAXIMUM);
        } else if (objective == "average") {
            solver.init(i, MakeSpanType::AVERAGE);
        }
        double approx = 0;
        if (i == 0 || window < std::numeric_limits<unsigned int>::max() / 2) {
            do {
                solver.initCBS(window);
                while (solver.step()) {

                }
                approx = solver.approxAverageMakeSpan(*solver.solution);
//                std::cout << "timestep: " << solver.currentTimestep - 1 << " " << approx << std::endl;
            } while (solver.simulate());
        } else {
            approx = solver.approxAverageMakeSpan(*solver.solution);
            solver.simulate();
        }
        out << solver.averageMakeSpan() << "," << approx << std::endl;
    }*/

    if (fout.is_open()) {
        fout.close();
    }

    return 0;
}
