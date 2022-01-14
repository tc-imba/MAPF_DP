#include <iostream>
#include <fstream>
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
    optionParser.add("random", false, 1, 0, "Map type (random / warehouse)", "-m", "--map");
    optionParser.add("maximum", false, 1, 0, "Objective type (maximum / average)", "--objective");
    optionParser.add("", false, 1, 0, "Output Filename", "-o", "--output");

    auto validWindowSize = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Window Size (0 means no limit)", "-w", "--window", validWindowSize);

    auto validSeed = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("0", false, 1, 0, "Simulation Seed", "-s", "--seed", validSeed);

    auto validAgents = new ez::ezOptionValidator("u4", "ge", "1");
    optionParser.add("35", false, 1, 0, "Agents Number", "-a", "--agents", validAgents);

    auto validIteration = new ez::ezOptionValidator("u4", "ge", "0");
    optionParser.add("10", false, 1, 0, "Iteration Number", "-i", "--iteration", validIteration);

    auto validMin = new ez::ezOptionValidator("d", "ge", "0");
    auto validMax = new ez::ezOptionValidator("d", "ge", "0");
    optionParser.add("0", false, 1, 0, "Min Delay Probability", "--min", validMin);
    optionParser.add("0.5", false, 1, 0, "Min Delay Probability", "--max", validMax);

    optionParser.parse(argc, argv);
    if (optionParser.isSet("-h")) {
        std::string usage;
        optionParser.getUsage(usage, 80, ez::ezOptionParser::ALIGN);
        std::cout << usage;
        return 1;
    }

    std::string mapType, objective, outputFileName;
    unsigned long window, seed, agentNum, iteration;
    double minDP, maxDP;
    bool debug, allConstraint;
    optionParser.get("--map")->getString(mapType);
    optionParser.get("--objective")->getString(objective);
    optionParser.get("--output")->getString(outputFileName);
    optionParser.get("--window")->getULong(window);
    optionParser.get("--seed")->getULong(seed);
    optionParser.get("--agents")->getULong(agentNum);
    optionParser.get("--iteration")->getULong(iteration);
    optionParser.get("--min")->getDouble(minDP);
    optionParser.get("--max")->getDouble(maxDP);
    debug = optionParser.isSet("--debug");
    allConstraint = optionParser.isSet("--all");

    if (window == 0) {
        window = std::numeric_limits<unsigned int>::max() / 2;
    }
//    std::cout << "window: " << window << std::endl;

    std::ofstream fout;
    if (!outputFileName.empty()) {
        fout.open(outputFileName);
        std::cerr << outputFileName << std::endl;
    }
    std::ostream &out = fout.is_open() ? fout : std::cout;


    Graph graph;
    unsigned int height = 30;
    unsigned int width = 30;
    unsigned int obstacles = 90;

    unsigned int deliveryWidth = 10;
    unsigned int deliveryX = 7;
    unsigned int deliveryY = 4;
    unsigned int maxX = 3 * deliveryX + 1;
    unsigned int maxY = deliveryY * (deliveryWidth + 1) + 13;

//    unsigned int agents = 30;
    graph.initDelayProbability(minDP, maxDP);


    std::string filename;
    if (mapType == "random") {
        filename = "random-" + std::to_string(height) + "-" + std::to_string(width) + "-" + std::to_string(obstacles);
        graph.generateRandomGraph(height, width, obstacles, filename, seed);
    } else if (mapType == "warehouse") {
        filename = "warehouse-" + std::to_string(maxX) + "-" + std::to_string(maxY);
        graph.generateWareHouse(deliveryWidth, maxX, maxY, filename, seed);
    } else if (mapType == "hardcoded") {
        filename = "hardcoded";
        graph.generateHardCodedGraph(filename, seed);
        agentNum = 2;
        seed = 1;
    }
    graph.calculateAllPairShortestPath(filename, true);


    std::vector<Agent> agents;
    if (mapType == "random") {
        agents = graph.generateRandomAgents(agentNum, seed);
    } else if (mapType == "warehouse") {
        agents = graph.generateWarehouseAgents(agentNum, seed, true);
    } else if (mapType == "hardcoded") {
        agents = graph.generateWarehouseAgents(agentNum, seed, false);
    }

    CBSSolver solver(graph, agents);
    solver.debug = debug;
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
    while (solver.step()) {}
    double approx = solver.approxAverageMakeSpan(*solver.solution);
    std::unique_ptr<Simulator> simulator;

    for (unsigned int i = 0; i < iteration; i++) {
//        simulator = std::make_unique<OnlineSimulator>(graph, agents, i);
        simulator = std::make_unique<DefaultSimulator>(graph, agents, i);
        simulator->setSolution(solver.solution);
        unsigned int currentTimestep = 1;
        bool unfinish = simulator->simulate(currentTimestep, currentTimestep + 1000);
        if (!unfinish) {
            out << simulator->averageMakeSpan(makeSpanType) << "," << approx << std::endl;
        }
    }



//    for (unsigned int i = 0; i < iteration; i++) {
//        if (objective == "maximum") {
//            solver.init(i, MakeSpanType::MAXIMUM);
//        } else if (objective == "average") {
//            solver.init(i, MakeSpanType::AVERAGE);
//        }
//        double approx = 0;
//        if (i == 0 || window < std::numeric_limits<unsigned int>::max() / 2) {
//            do {
//                solver.initCBS(window);
//                while (solver.step()) {
//
//                }
//                approx = solver.approxAverageMakeSpan(*solver.solution);
////                std::cout << "timestep: " << solver.currentTimestep - 1 << " " << approx << std::endl;
//            } while (solver.simulate());
//        } else {
//            approx = solver.approxAverageMakeSpan(*solver.solution);
//            solver.simulate();
//        }
//        out << solver.averageMakeSpan() << "," << approx << std::endl;
//    }

    if (fout.is_open()) {
        fout.close();
    }

    return 0;
}
