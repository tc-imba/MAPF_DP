//
// Created by liu on 18/12/2022.
//

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string_regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#include <fstream>
#include <iostream>
#include <limits>

//#include "driver.h"
#include "EECBSSolver.h"

bool EECBSSolver::readSolution(std::ifstream &fin) {
    if (!fin.is_open()) return false;

    auto node = std::make_shared<CBSNode>();
    node->constraint = {INVALID, INVALID, INVALID};
    node->plans.resize(agents.size());
    node->parent = nullptr;
    node->depth = 0;

    for (size_t i = 0; i < agents.size(); i++) {
        node->plans[i] = std::make_shared<AgentPlan>();
        std::string line;
        if (!std::getline(fin, line)) return false;
        line = line.substr(8 + std::to_string(i).size());
        std::vector<std::string> points;
        boost::algorithm::split_regex(points, line, boost::regex("->"));
        for (auto &point: points) {
            if (point.empty()) continue;
            point = point.substr(1, point.size() - 2);
            std::vector<std::string> coordinates;
            boost::split(coordinates, point, boost::is_any_of(","));
            if (coordinates.size() != 2) return false;
            unsigned int x = std::stoul(coordinates[0]);
            unsigned int y = std::stoul(coordinates[1]);
            auto nodeId = graph.getNodeIdByGridPos(x, y);
            if (nodeId == std::numeric_limits<unsigned int>::max()) return false;
            node->plans[i]->add(nodeId);
        }
        for (size_t j = 0; j < node->plans[i]->path.size(); j++) {
            node->plans[i]->path[j].estimatedTime = j;
            node->plans[i]->path[j].heuristic = node->plans[i]->path.size() - j - 1;
        }
    }
    solution = node;
    return true;
}


bool EECBSSolver::solve() {
    using namespace boost::filesystem;
    using namespace boost::process;

    executionTime = 0;

    path ph = path("eecbs_results") / unique_path();
    create_directories(ph);

    std::string mapFileName = graph.getFileName() + ".map";
    path scenFile = ph / "tasks";
    std::string scenFileName = scenFile.string() + ".scen";
    std::string outputFileName = (ph / "output.csv").string();
    std::string pathFileName = (ph / "path.txt").string();
    std::string commandFilename = (ph / "command.txt").string();

    graph.saveScenAgents(mapFileName, scenFile.string(), agents);

    std::vector<std::string> arguments;
    arguments.emplace_back(solverBinaryFile);
    arguments.emplace_back("-m");
    arguments.emplace_back(mapFileName);
    arguments.emplace_back("-a");
    arguments.emplace_back(scenFileName);
    arguments.emplace_back("-o");
    arguments.emplace_back(outputFileName);
    arguments.emplace_back("--outputPaths");
    arguments.emplace_back(pathFileName);
    arguments.emplace_back("-k");
    arguments.emplace_back(std::to_string(agents.size()));
    arguments.emplace_back("-t");
    arguments.emplace_back("60");
    arguments.emplace_back("--suboptimality");
    arguments.emplace_back(std::to_string(suboptimality));

    if (prioritizedReplan) {
        std::string obstaclesFileName = scenFile.string() + ".obs";
        saveObstacles(obstaclesFileName);
        arguments.emplace_back("--dynamicObstacles");
        arguments.emplace_back(obstaclesFileName);
    }

    std::string argumentsStr = boost::algorithm::join(arguments, " ");
    std::ofstream fout(commandFilename);
    fout << argumentsStr << std::endl;
    fout.close();

    child c(argumentsStr, std_out > null, std_err > null);
    auto start = std::chrono::steady_clock::now();
    c.wait();

    //    eecbs((int) cstrings.size(), cstrings.data());

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    executionTime = elapsed_seconds.count();

    std::ifstream fin(pathFileName);
    success = readSolution(fin);
    fin.close();

    //    if (!prioritizedReplan) {
    remove_all(ph);
    //    }
    //    prioritizedReplan = false;
    //    std::cerr << prioritizedReplan << " " << success << std::endl;

    return success;
}


void EECBSSolver::saveObstacles(const std::string &filename) {
    std::vector<std::tuple<unsigned int, unsigned int, int, int>> result;
    for (size_t i = 0; i < savedAgents.size(); i++) {
        if (savedPlans[i]) {
            auto &path = savedPlans[i]->path;
            for (size_t j = 0; j < path.size(); j++) {
                auto &node = graph.getNode(path[j].nodeId);
                auto tmax = (j == path.size() - 1) ? INT_MAX / 2 : j + 1;
                result.emplace_back(node.x, node.y, j, tmax);
            }
        }
    }
    std::ofstream fileOut(filename);
    fileOut << result.size() << std::endl;
    for (const auto &row: result) {
        fileOut << std::get<0>(row) << " " << std::get<1>(row) << " " << std::get<2>(row) << " " << std::get<3>(row)
                << std::endl;
    }
    fileOut.close();
}
