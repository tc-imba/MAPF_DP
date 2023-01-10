//
// Created by liu on 27/4/2021.
//

#include "Solver.h"
#include <random>
#include <iostream>
#include <fstream>

void Solver::init() {
    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].current = agents[i].start;
        agents[i].timestep = agents[i].waitingTimestep = 0;
    }
    currentTimestep = 1;
    success = false;
}

Solver::Solver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType) :
        graph(graph), agents(agents), makeSpanType(makeSpanType) {

}


size_t Solver::combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int seed) {
    size_t result = 0;
    boost::hash_combine(result, nodeId1 ^ nodeId2);
    boost::hash_combine(result, timestep);
    boost::hash_combine(result, seed);
    return result;
}


double Solver::approxAverageMakeSpan(CBSNode &cbsNode) {
    double result = 0;
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (makeSpanType == MakeSpanType::MAXIMUM) {
            if (agents[i].current == agents[i].goal && cbsNode.plans[i]->path.size() == 1) {
                result = std::max(result, (double) agents[i].timestep);
            } else if (!cbsNode.plans[i]->path.empty()) {
                result = std::max(result, currentTimestep - 1 + cbsNode.plans[i]->path.back().estimatedTime);
            } else {
                result = std::max(result, 1e9);
            }
        } else if (makeSpanType == MakeSpanType::AVERAGE) {
            if (agents[i].current == agents[i].goal && cbsNode.plans[i]->path.size() == 1) {
                result += agents[i].timestep / (double) agents.size();
            } else if (!cbsNode.plans[i]->path.empty()) {
                result += (currentTimestep - 1 + cbsNode.plans[i]->path.back().estimatedTime) / (double) agents.size();
            } else {
                result = std::max(result, 1e9);
            }
        }
    }
    return result;
}

bool Solver::solveWithCache(const std::string &filename, unsigned int agentSeed) {
    std::string cacheFilename =
            filename + "-" +
            std::to_string(agents.size()) + "-" +
            std::to_string(agentSeed) + "-" +
            getSolverName() + ".cbs";
    //    std::cerr << cacheFilename << std::endl;
    std::ifstream fin(cacheFilename);
    if (fin.is_open()) {
        solution = std::make_shared<CBSNode>();
        for (unsigned int i = 0; i < agents.size(); i++) {
            solution->plans.emplace_back(std::make_shared<AgentPlan>());
            unsigned int pathLength;
            fin >> pathLength;
            for (unsigned int j = 0; j < pathLength; j++) {
                Label label{};
                fin >> label.nodeId >> label.state >> label.estimatedTime >> label.heuristic;
                solution->plans[i]->path.push_back(label);
            }
        }
        return true;
    }
    fin.close();
    if (!solve()) {
        std::cout << "failed" << std::endl;
        return false;
    }
    std::ofstream fout(cacheFilename);
    if (fout.is_open()) {
        for (unsigned int i = 0; i < agents.size(); i++) {
            unsigned int pathLength = solution->plans[i]->path.size();
            fout << pathLength << std::endl;
            for (unsigned int j = 0; j < pathLength; j++) {
                auto &label = solution->plans[i]->path[j];
                fout << label.nodeId << " "
                     << label.state << " "
                     << label.estimatedTime << " "
                     << label.heuristic << std::endl;
            }
        }
    }
    return success;
}
