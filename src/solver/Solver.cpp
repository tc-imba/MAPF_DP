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
        agents[i].timestep = agents[i].arrivingTimestep = 0;
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

bool Solver::validate() {
    std::map<unsigned int, unsigned int> last;
    unsigned int followingConflicts = 0, cycleConflicts = 0;

    for (unsigned int state = 0;; state++) {
        int count = 0;
        std::map<unsigned int, unsigned int> following;
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &path = solution->plans[i]->path;
            if (state < path.size()) {
                auto it = last.find(path[state].nodeId);
                if (it != last.end() && i != it->second) {
                    following.emplace(i, it->second);
                }
            } else {
                count++;
            }
        }
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &path = solution->plans[i]->path;
            if (state < path.size()) {
                if (state > 0) {
                    last.erase(path[state - 1].nodeId);
                }
                last[path[state].nodeId] = i;
            }
        }

        followingConflicts += following.size();
        std::set<unsigned int> cycle;

        while (!following.empty()) {
            auto it = following.begin();
            auto src = it->first;
            auto current = it->second;
            following.erase(it);
            while (true) {
                it = following.find(current);
                if (it != following.end()) {
                    current = it->second;
                    following.erase(it);
                    if (src == current) {
                        cycleConflicts += 1;
                    }
                } else {
                    break;
                }
            }
        }

        if (count == agents.size()) {
            break;
        }
    }

    return cycleConflicts == 0;
}


bool Solver::solveWithPrioritizedReplan() {
    prioritizedReplan = true;
    savedAgents = std::move(agents);
    savedPlans.clear();
    savedPlans.resize(savedAgents.size());
    plannedAgentsMap.resize(savedAgents.size());
    agents.clear();
//    std::cerr << "before: " << std::endl;
    for (size_t i = 0; i < savedAgents.size(); i++) {
        auto &agent = savedAgents[i];
        auto &path = solution->plans[i]->path;
//        std::cerr << path.size() - agent.state << " ";
        if (agent.state + 1 >= path.size() || !agent.delayed) {
            size_t distance = std::min((size_t) agent.state, path.size() - 1);
            path.erase(path.begin(), path.begin() + distance);
            for (size_t j = 0; j < path.size(); j++) {
                path[j].state = j;
            }
            savedPlans[i] = solution->plans[i];
            plannedAgentsMap[i] = savedAgents.size();
        } else {
            savedPlans[i] = nullptr;
            plannedAgentsMap[i] = agents.size();
            agents.push_back(savedAgents[i]);
        }
    }
//    std::cerr << std::endl;
//    std::cerr << "replan: " << agents.size() << std::endl;
    if (!agents.empty()) {
        if (!solve()) {
            agents = std::move(savedAgents);
            return solveRetry();
        }
    }
//    std::cerr << "after: " << std::endl;
    agents = std::move(savedAgents);
    for (size_t i = 0; i < agents.size(); i++) {
        if (!savedPlans[i]) {
            savedPlans[i] = solution->plans[plannedAgentsMap[i]];
        }
//        std::cerr << savedPlans[i]->path.size() << " ";
    }
//    std::cerr << std::endl;
    solution->plans = std::move(savedPlans);
    if (!validate()) {
        return solveRetry();
    }
    return true;
}

bool Solver::solveRetry() {
    auto lastExecutionTime = executionTime;
//    std::cerr << "prioritized replan failed, retry with full replan" << std::endl;
    prioritizedReplan = false;
    init();
    auto result = solve();
    executionTime += lastExecutionTime;
    return result;
}