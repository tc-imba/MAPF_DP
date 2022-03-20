//
// Created by liu on 7/1/2022.
//

#include <boost/range/join.hpp>
#include <iostream>
#include <random>

#include "OnlineSimulator.h"

int OnlineSimulator::simulate(unsigned int &currentTimestep, unsigned int maxTimeStep, unsigned int pauseTimestep) {
    initSimulation();

    std::uniform_real_distribution<double> distribution(0, 1);

    for (; currentTimestep < maxTimeStep; currentTimestep++) {
        if (debug) {
            std::cout << "begin timestep " << currentTimestep << std::endl;
        }

        if (delayInterval > 0 && currentTimestep % delayInterval == 0) {
            delayedSet.clear();
        }

        initChecks();
        neighborCheck();
        deadEndCheck();

        unblocked.insert(ready.begin(), ready.end());
        ready.clear();

//        if (pauseTimestep > 0 && currentTimestep == pauseTimestep) {
//            updateDelayedSet(currentTimestep);
//        }
        updateDelayedSet(currentTimestep);

        int count = 0;
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            auto currentNodeId = paths[i][state];
            if (state + 1 >= paths[i].size()) {
                if (debug) {
                    std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") completed" << std::endl;
                }
                ++count;
            } else if (delayedSet.find(i) != delayedSet.end()) {
                if (debug) {
                    std::cout << "agent " << i << ": delayed" << std::endl;
                }
                agents[i].timestep = currentTimestep;
            } else if (blocked.find(i) != blocked.end()) {
                if (debug) {
                    std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") blocked" << std::endl;
                }
                agents[i].timestep = currentTimestep;
            } else if (unblocked.find(i) != unblocked.end()) {
                agents[i].timestep = currentTimestep;
                auto nextNodeId = paths[i][state + 1];
                auto &edge = graph.getEdge(currentNodeId, nextNodeId);
                auto waitingTimestep = 1;
//                auto waitingTimestep = (unsigned int) floor(10.0 * (edge.dp - 0.5) + 2);
//                auto waitingTimestep = (unsigned int) floor(1 / (1.0 - edge.dp));
//                auto waitingTimestep = (unsigned int) floor(exp(5.0 * (edge.dp - 0.5)) + 1);
                if (++agents[i].waitingTimestep < waitingTimestep) {
//                std::mt19937 generator(combineRandomSeed(currentNodeId, nextNodeId, currentTimestep, seed));
//                double rand = distribution(generator);
//                if (rand < edge.dp) {
                    if (debug) {
                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") delay" << std::endl;
                    }
                    nodeAgentMap[nextNodeId] = i;
                } else {
                    if (debug) {
                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ")->("
                                  << state + 1 << "," << nextNodeId << ")" << std::endl;
                    }
                    unblocked.erase(i);
                    moved.insert(i);
                    nodeAgentMap.erase(currentNodeId);
                    nodeAgentMap[nextNodeId] = i;
                    agents[i].current = nextNodeId;
                    agents[i].waitingTimestep = 0;
                    if (++state >= paths[i].size()) ++count;
                }
            } else {
                assert(0);
            }
        }
        if (count >= agents.size()) {
            break;
        }

    }

/*    bool unfinish = false;
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (agents[i].current != agents[i].goal) {
            unfinish = true;
//            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << ", goal " << agents[i].goal << std::endl;
            break;
        }
    }*/
    return countCompletedAgents();
}


void OnlineSimulator::initSharedNodes(size_t i, size_t j) {
    std::unordered_map<unsigned int, std::vector<std::pair<size_t, unsigned int> > > m;
    for (auto _i: {i, j}) {
        for (size_t k = 0; k < paths[_i].size(); k++) {
            m[paths[_i][k]].emplace_back(_i, k);
        }
    }
    // remove unshared nodes
    for (auto it = m.begin(); it != m.end();) {
        bool ai = false, aj = false;
        for (auto &&[agentId, state]: it->second) {
            if (agentId == i) ai = true;
            if (agentId == j) aj = true;
        }
        if (ai && aj) {
            ++it;
        } else {
            it = m.erase(it);
        }
    }
    // init dead-end states
    for (auto[_i, _j]: {std::make_pair(i, j),
                        std::make_pair(j, i)}) {
        auto lastNodeId = paths[_i].back();
        if (m.find(lastNodeId) != m.end()) {
            unsigned int lastState = 0;
            for (auto &&[agentId, state]: m[lastNodeId]) {
                if (agentId == _j) {
                    lastState = std::max(lastState, state);
                }
            }
            deadEndStates[_i].emplace_back(_j, lastState);
        }
    }

}


void OnlineSimulator::initSimulation() {
    blocked.clear();
    paths.clear();
    paths.resize(agents.size());
    deadEndStates.clear();
    deadEndStates.resize(agents.size());
    nodeAgentMap.clear();
    for (size_t i = 0; i < agents.size(); i++) {
        blocked.insert(i);
        nodeAgentMap[agents[i].current] = i;
        for (size_t j = 0; j < solution->plans[i]->path.size(); j++) {
            auto newNodeId = solution->plans[i]->path[j].nodeId;
            if (paths[i].empty() || paths[i].back() != newNodeId) {
                paths[i].emplace_back(newNodeId);
            }
        }
    }
    for (size_t i = 0; i < agents.size(); i++) {
        for (size_t j = i + 1; j < agents.size(); j++) {
            initSharedNodes(i, j);
        }
    }
}


void OnlineSimulator::initChecks() {
    ready.clear();
    for (auto i: boost::join(blocked, moved)) {
        auto &agent = agents[i];
        if (agent.current != agent.goal && agent.state + 1 < paths[i].size()) {
            ready.insert(i);
        }
    }
    blocked.clear();
    moved.clear();
//    for (auto i: delayed_set) {
//        if (ready.find(i) != ready.end()) {
//            ready.erase(i);
//            blocked.insert(i);
//        }
//    }
}


void OnlineSimulator::neighborCheck() {
    for (auto it = ready.begin(); it != ready.end();) {
        auto &agent = agents[*it];
        auto nextNodeId = paths[*it][agent.state + 1];
        auto it2 = nodeAgentMap.find(nextNodeId);
        if (it2 != nodeAgentMap.end() && *it != it2->second) {
            blocked.insert(*it);
            it = ready.erase(it);
        } else {
            ++it;
        }
    }
}

void OnlineSimulator::deadEndCheck() {
    for (auto it = ready.begin(); it != ready.end();) {
        bool remove = false;
        if (agents[*it].state == paths[*it].size() - 2) {
            for (auto &&[j, state]: deadEndStates[*it]) {
                if (agents[j].state <= state) {
                    remove = true;
                    break;
                }
            }
        }
        if (remove) {
            blocked.insert(*it);
            it = ready.erase(it);
        } else {
            ++it;
        }
    }
}

void OnlineSimulator::cycleCheck() {



}




