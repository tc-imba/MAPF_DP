//
// Created by liu on 27/4/2021.
//

#include <random>
#include <iostream>
#include <fstream>

#include "Solver.h"
#include "../simulator/DiscreteDefaultSimulator.h"

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


bool Solver::removeFollowingConflict(std::shared_ptr<Solver> solver) {
    auto simulator = std::make_unique<DiscreteDefaultSimulator>(solver->graph, solver->agents, 0);
    simulator->delayRatio = 0;
    simulator->delayType = "agent";
    simulator->setSolver(solver);
    simulator->debug = false;

    double currentTimestep = 0;
    auto count = simulator->simulate(currentTimestep, 1000, 0, 0);
    if (count != solver->agents.size()) return false;

    for (size_t i = 0; i < solver->agents.size(); i++) {
        /*        std::cout << i << std::endl;
        for (auto loc: solution->plans[i]->path) {
            std::cout << loc.nodeId << " ";
        }
        std::cout << std::endl;
        for (auto loc: simulator->agentRealPaths[i]) { std::cout << loc << " "; }
        std::cout << std::endl;*/
        auto &realPath = simulator->agentRealPaths[i];
        auto &path = solver->solution->plans[i]->path;
        while (realPath.size() > 1) {
            if (realPath[realPath.size() - 1] == realPath[realPath.size() - 2]) {
                realPath.pop_back();
            } else
                break;
        }
        path.clear();
        for (unsigned int j = 0; j < realPath.size(); j++) {
            path.emplace_back(Label{realPath[j], j, (double) (realPath.size() - j), (double) j});
        }
        /*        for (auto loc: solution->plans[i]->path) {
            std::cout << loc.nodeId << " ";
        }
        std::cout << std::endl;*/
    }

    return true;
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

bool Solver::solveWithCache(std::shared_ptr<Solver> solver, const std::string &filename, unsigned int agentSeed) {
    std::string cacheFilename =
            filename + "-" +
            std::to_string(solver->agents.size()) + "-" +
            std::to_string(agentSeed) + "-" +
            solver->getSolverName() + ".cbs";
    //    std::cerr << cacheFilename << std::endl;
    std::ifstream fin(cacheFilename);
    if (fin.is_open()) {
        SPDLOG_INFO("use solver cache: {}", cacheFilename);
        solver->solution = std::make_shared<CBSNode>();
        for (unsigned int i = 0; i < solver->agents.size(); i++) {
            solver->solution->plans.emplace_back(std::make_shared<AgentPlan>());
            unsigned int pathLength;
            fin >> pathLength;
            for (unsigned int j = 0; j < pathLength; j++) {
                Label label{};
                fin >> label.nodeId >> label.state >> label.estimatedTime >> label.heuristic;
                solver->solution->plans[i]->path.push_back(label);
            }
        }
        return true;
    }
    SPDLOG_INFO("no cache: {}", cacheFilename);
    fin.close();
    if (!solver->solve()) {
        std::cout << "failed" << std::endl;
        return false;
    }
    removeFollowingConflict(solver);
    std::ofstream fout(cacheFilename);
    if (fout.is_open()) {
        for (unsigned int i = 0; i < solver->agents.size(); i++) {
            unsigned int pathLength = solver->solution->plans[i]->path.size();
            fout << pathLength << std::endl;
            for (unsigned int j = 0; j < pathLength; j++) {
                auto &label = solver->solution->plans[i]->path[j];
                fout << label.nodeId << " "
                     << label.state << " "
                     << label.estimatedTime << " "
                     << label.heuristic << std::endl;
            }
        }
    }
    return solver->success;
}

bool Solver::validate(unsigned int maxState) {
    // nodeId -> agentId, in current state, the last agent at a node
    std::map<unsigned int, unsigned int> last;
    // the deadend nodes before current state
    std::set<unsigned int> deadends;
    unsigned int followingConflicts = 0, cycleConflicts = 0, deadendConflicts = 0;

    for (unsigned int state = 0;; state++) {
        size_t count = 0;
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
                if (deadends.find(path[state].nodeId) != deadends.end()) {
                    deadendConflicts += 1;
                }
                if (state == path.size() - 1) {
                    deadends.emplace(path[state].nodeId);
                }
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

        if (state >= maxState) {
            break;
        }
    }

//    return cycleConflicts == 0;

    return cycleConflicts == 0 && deadendConflicts == 0;
}


bool Solver::solveWithPrioritizedReplan(bool prioritizedOpt) {
    partialExecutionTime = 0;
    executionTime = 0;
    prioritizedReplan = true;
    savedAgents = std::move(agents);
    savedPlans.clear();
    savedPlans.resize(savedAgents.size());
    plannedAgentsMap.resize(savedAgents.size());
    agents.clear();
//    std::cerr << "before: " << std::endl;
//    for (size_t i = 0; i < savedAgents.size(); i++) {
//
//    }
    for (size_t i = 0; i < savedAgents.size(); i++) {
        auto &agent = savedAgents[i];
        auto &path = solution->plans[i]->path;
//        std::cerr << path.size() - agent.state << " ";
        if (agent.state + 1 >= path.size() || !agent.delayed) {
//        if (agent.state + 1 < path.size() && !agent.delayed) {
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
//    if (agents.size() == savedAgents.size()) {
//        prioritizedReplan = false;
//    }
    if (!agents.empty()) {
        if (!solve()) {
//            std::cerr << "solve failed, retry with full replan" << std::endl;
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
    unsigned int maxState = prioritizedOpt ? 3 : std::numeric_limits<unsigned int>::max();
    if (!validate(maxState)) {
//        std::cerr << "validate failed, retry with full replan" << std::endl;
        return solveRetry();
    }
    partialExecutionTime = executionTime;
    executionTime = 0;
    prioritizedReplan = false;
    return true;
}

bool Solver::solveRetry() {
//    std::cerr << "prioritized replan failed, retry with full replan" << std::endl;
    partialExecutionTime = executionTime;
    prioritizedReplan = false;
    init();
    return solve();
}
