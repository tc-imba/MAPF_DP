//
// Created by liu on 7/1/2022.
//

#include <boost/range/join.hpp>
#include <boost/graph/topological_sort.hpp>
#include <iostream>
#include <random>

#include "OnlineSimulator.h"

int OnlineSimulator::simulate(unsigned int &currentTimestep, unsigned int maxTimeStep, unsigned int pauseTimestep) {
    initSimulation();

    std::uniform_real_distribution<double> distribution(0, 1);

    if (debug) {
        for (unsigned int i = 0; i < agents.size(); i++) {
//        if (agents[i].current == agents[i].goal) continue;
            std::cout << "agent " << i << "(" << agents[i].start << "->" << agents[i].goal << "): ";
            for (const auto &label: solution->plans[i]->path) {
                std::cout << "(" << label.state << "," << label.nodeId << ")->";
            }
            std::cout << std::endl;
//        nodeStates[agents[i].start] = 0;
        }
    }


    for (; currentTimestep < maxTimeStep; currentTimestep++) {
        if (debug) {
            std::cout << "begin timestep " << currentTimestep << std::endl;
        }

        if (pauseTimestep == 0 && delayInterval > 0 && currentTimestep % delayInterval == 0) {
            delayedSet.clear();
        }

        initChecks();
        if (!isOnlyCycleCheck) {
            unsharedCheck();
            neighborCheck();
            deadEndCheck();
        }
        cycleCheck();

        unblocked.insert(ready.begin(), ready.end());
        unblocked.insert(unshared.begin(), unshared.end());
        ready.clear();
        unshared.clear();

        if (firstAgentArrivingTimestep == 0) {
            unblockedAgents += unblocked.size();
        }

        if ((pauseTimestep == 0 && delayInterval > 0) || currentTimestep == pauseTimestep) {
            updateDelayedSet(currentTimestep, pauseTimestep == 0);
        }

//        if (pauseTimestep > 0 && currentTimestep == pauseTimestep) {
//            updateDelayedSet(currentTimestep);
//        }
//        updateDelayedSet(currentTimestep);

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
                    std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") delayed" << std::endl;
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
                    updateSharedNode(currentNodeId, i, state);
                    agents[i].current = nextNodeId;
                    agents[i].waitingTimestep = 0;
                    state++;
                    if (state + 1 >= paths[i].size()) {
                        if (firstAgentArrivingTimestep == 0) {
                            firstAgentArrivingTimestep = currentTimestep;
                        }
                        ++count;
                    }
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

void OnlineSimulator::print(std::ostream &out) const {
    out << feasibilityCheckCount << "," << ((double) cycleCheckAgents) / ((double) firstAgentArrivingTimestep);
    out << "," << ((double) unblockedAgents) / ((double) firstAgentArrivingTimestep);
    for (int i = 0; i < 4; i++) {
        out << "," << feasibilityCheckTypes[i];
    }
}

void OnlineSimulator::initSharedNodes(size_t i, size_t j) {
    std::unordered_map<unsigned int, std::vector<std::pair<size_t, unsigned int> > > m;
    for (auto _i: {i, j}) {
        for (size_t k = 0; k < paths[_i].size(); k++) {
            m[paths[_i][k]].emplace_back(_i, k);
        }
    }
    // remove unshared nodes and init shared nodes
    for (auto it = m.begin(); it != m.end();) {
        std::vector<unsigned int> vi, vj;
        for (auto &&[agentId, state]: it->second) {
            if (agentId == i) vi.emplace_back(state);
            if (agentId == j) vj.emplace_back(state);
        }
        if (!vi.empty() && !vj.empty()) {
            auto &sharedNode = sharedNodes[it->first];
            for (auto state1: vi) {
                for (auto state2: vj) {
                    sharedNode.emplace_back(SharedNodePair{i, state1, j, state2});
                }
            }
            ++it;
        } else {
            it = m.erase(it);
        }
    }
    // init dead-end states
    for (auto [_i, _j]: {std::make_pair(i, j),
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

void OnlineSimulator::updateSharedNode(unsigned int nodeId, size_t agentId, unsigned int state) {
    auto it = sharedNodes.find(nodeId);
    if (it == sharedNodes.end()) {
        return;
    }
    for (auto it2 = it->second.begin(); it2 != it->second.end();) {
        if ((it2->agentId1 == agentId && it2->state1 <= state) || (it2->agentId2 == agentId && it2->state2 <= state)) {
            it2 = it->second.erase(it2);
        } else {
            ++it2;
        }
    }
}


void OnlineSimulator::initSimulation() {
    blocked.clear();
    paths.clear();
    paths.resize(agents.size());
    deadEndStates.clear();
    deadEndStates.resize(agents.size());
    pathTopoNodeIds.clear();
    pathTopoNodeIds.resize(agents.size());
    nodeAgentMap.clear();
    delayedSet.clear();
    topoGraph.clear();

    unsigned int topoGraphNodeNum = 0;
    for (size_t i = 0; i < agents.size(); i++) {
        blocked.insert(i);
        nodeAgentMap[agents[i].current] = i;
        for (size_t j = 0; j < solution->plans[i]->path.size(); j++) {
            auto newNodeId = solution->plans[i]->path[j].nodeId;
            if (paths[i].empty() || paths[i].back() != newNodeId) {
                paths[i].emplace_back(newNodeId);
                pathTopoNodeIds[i].emplace_back(boost::add_vertex(topoGraph));
            }
        }
        for (size_t j = 0; j < paths[i].size() - 1; j++) {
            boost::add_edge(pathTopoNodeIds[i][j], pathTopoNodeIds[i][j + 1], topoGraph);
        }
        topoGraphNodeNum += paths[i].size();
    }


    for (size_t i = 0; i < agents.size(); i++) {
        for (size_t j = i + 1; j < agents.size(); j++) {
            initSharedNodes(i, j);
        }
    }

    feasibilityCheckCount = 0;
    cycleCheckAgents = 0;
    unblockedAgents = 0;
    firstAgentArrivingTimestep = 0;
    for (int i = 0; i < 4; i++) {
        feasibilityCheckTypes[i] = 0;
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
    unshared.clear();
//    for (auto i: delayed_set) {
//        if (ready.find(i) != ready.end()) {
//            ready.erase(i);
//            blocked.insert(i);
//        }
//    }
}

void OnlineSimulator::unsharedCheck() {
    for (auto it = ready.begin(); it != ready.end();) {
        auto &agent = agents[*it];
        auto nextNodeId = paths[*it][agent.state + 1];
        auto it2 = sharedNodes.find(nextNodeId);
        if (it2 == sharedNodes.end() || it2->second.size() <= 1) {
            unshared.insert(*it);
            it = ready.erase(it);
        } else {
            ++it;
        }
    }
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

void OnlineSimulator::naiveCycleCheckHelper(std::vector<size_t> &readyList, size_t length, size_t start, size_t current,
                                            std::vector<bool> &check, std::vector<size_t> &maxReadyList) {
    if (length <= maxReadyList.size()) return;
    if (current > length) return;
    if (current == length) {
        std::vector<size_t> checkedReady;
        for (size_t i = 0; i < readyList.size(); i++) {
            if (check[i]) {
                checkedReady.emplace_back(readyList[i]);
            }
        }
        if (checkedReady.size() <= maxReadyList.size()) return;
        for (auto i: checkedReady) {
            agents[i].state++;
        }
        auto [agentId1, agentId2] = feasibilityCheck();
        for (auto i: checkedReady) {
            agents[i].state--;
        }
        if (agentId1 == agents.size() && agentId2 == agents.size()) {
            maxReadyList.swap(checkedReady);
        }
        return;
    }
    if (start == readyList.size()) return;
    check[start] = true;
    naiveCycleCheckHelper(readyList, length, start + 1, current + 1, check, maxReadyList);
    check[start] = false;
    naiveCycleCheckHelper(readyList, length, start + 1, current, check, maxReadyList);
}

void OnlineSimulator::naiveCycleCheck() {
//    std::cerr << ready.size() << std::endl;
    std::vector<size_t> readyList;
    readyList.insert(readyList.begin(), ready.begin(), ready.end());
    std::vector<bool> check(readyList.size(), false);
    std::vector<size_t> maxReadyList;
    for (size_t i = 1; i <= readyList.size(); i++) {
        naiveCycleCheckHelper(readyList, i, 0, 0, check, maxReadyList);
    }
    for (auto i: maxReadyList) {
        if (ready.find(i) == ready.end()) {
            blocked.insert(i);
        }
    }
    ready.clear();
    ready.insert(maxReadyList.begin(), maxReadyList.end());
}

void OnlineSimulator::heuristicCycleCheck() {
    //    std::cerr << "start cycle check: ";
    std::set<size_t> newReady = ready;
    for (auto i: newReady) {
        agents[i].state++;
//        std::cerr << i << " ";
    }
//    std::cerr << std::endl;

    // use feasibility check to block some agents
    std::set<size_t> removed;
    while (!newReady.empty()) {
//        std::cerr << "check 1" << std::endl;
        auto [agentId1, agentId2] = feasibilityCheck();
        if (agentId1 == agents.size() && agentId2 == agents.size()) {
            break;
        }
        // TODO: add randomness here
//        std::cerr << agentId1 << " " << agentId2 << std::endl;
        if (newReady.find(agentId1) != newReady.end()) {
            newReady.erase(agentId1);
            agents[agentId1].state--;
            removed.insert(agentId1);
        } else if (newReady.find(agentId2) != newReady.end()) {
            newReady.erase(agentId2);
            agents[agentId2].state--;
            removed.insert(agentId2);
        } else {
            for (auto i: newReady) {
                agents[i].state--;
                removed.insert(i);
            }
            newReady.clear();
//            std::cerr << "warn: nothing to remove!" << std::endl;
            break;
        }
    }

    // if all agents are blocked, try one by one to unblock
//    if (newReady.empty()) {
//        std::cerr << "warn: ready empty" << std::endl;
    for (auto i: ready) {
        if (newReady.find(i) == newReady.end()) {
            agents[i].state++;
//            std::cerr << "check 2: " << i << std::endl;
            auto [agentId1, agentId2] = feasibilityCheck();
            if (agentId1 == agents.size() && agentId2 == agents.size()) {
                newReady.insert(i);
                // break;
            } else {
                agents[i].state--;
            }
        }
    }
//    }

    // hope this not happen
    if (newReady.empty() && unblocked.empty() && unshared.empty()) {
        std::cerr << "error: ready and unblocked both empty!" << std::endl;
    }

    for (auto i: newReady) {
        agents[i].state--;
//        std::cerr << i << " ";
    }
//    std::cerr << std::endl;

    for (auto i: newReady) {
        if (ready.find(i) == ready.end()) {
            blocked.insert(i);
        }
    }
    ready = newReady;
}


void OnlineSimulator::cycleCheck() {
    if (ready.empty()) return;
    if (firstAgentArrivingTimestep == 0) {
        cycleCheckAgents += ready.size();
    }
    // suppose all unblocked agents are at their next state
    for (auto i: unblocked) {
        agents[i].state++;
    }

    if (isHeuristicCycleCheck) {
        heuristicCycleCheck();
    } else {
        naiveCycleCheck();
    }

    // recover the states of the agents
    for (auto i: unblocked) {
        agents[i].state--;
    }
//    std::cerr << "end cycle check: ";
}

std::pair<size_t, size_t> OnlineSimulator::feasibilityCheckHelper(
        std::list<SharedNodePair> &sharedNodesList,
        bool recursive
//        std::vector<std::pair<unsigned int, unsigned int>> &addedEdges
) {
    std::vector<std::pair<unsigned int, unsigned int>> addedEdges;

    while (!sharedNodesList.empty()) {
        unsigned int erasedEdges = 0;

        for (auto it = sharedNodesList.begin(); it != sharedNodesList.end();) {
            std::vector<std::pair<unsigned int, unsigned int>> selectedEdges;
            unsigned int maxSelectedEdges = 0;
            if (it->state1 + 1 < paths[it->agentId1].size() && it->state2 > agents[it->agentId2].state) {
                auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
                auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];
                boost::add_edge(nodeId1, nodeId2, topoGraph);
                std::vector<Graph::topo_vertex_t> container;
                try {
                    boost::topological_sort(topoGraph, std::back_inserter(container));
                    selectedEdges.emplace_back(nodeId1, nodeId2);
                } catch (std::exception) {}
                boost::remove_edge(nodeId1, nodeId2, topoGraph);
                ++maxSelectedEdges;
            }
            if (it->state2 + 1 < paths[it->agentId2].size() && it->state1 > agents[it->agentId1].state) {
                auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1];
                auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2 + 1];
                boost::add_edge(nodeId2, nodeId1, topoGraph);
                try {
                    std::vector<Graph::topo_vertex_t> container;
                    boost::topological_sort(topoGraph, std::back_inserter(container));
                    selectedEdges.emplace_back(nodeId2, nodeId1);
                } catch (std::exception) {}
                boost::remove_edge(nodeId2, nodeId1, topoGraph);
                ++maxSelectedEdges;
            }
            if (selectedEdges.empty()) {
                if (maxSelectedEdges > 0) {
                    // failed
                    // std::cerr << "failed!" << std::endl;
                    for (auto [nodeId1, nodeId2]: addedEdges) {
                        boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    }
                    return std::make_pair(it->agentId1, it->agentId2);
                }
                it = sharedNodesList.erase(it);
                ++erasedEdges;
            } else if (selectedEdges.size() == 1) {
                addedEdges.emplace_back(selectedEdges[0]);
                boost::add_edge(selectedEdges[0].first, selectedEdges[0].second, topoGraph);
                it = sharedNodesList.erase(it);
                ++erasedEdges;
//                std::cerr << "choose: " << selectedEdges[0].first << " " << selectedEdges[0].second << std::endl;
            } else {
                ++it;
            }
        }

        if (erasedEdges == 0 && !sharedNodesList.empty()) {
            auto it = sharedNodesList.begin();
            auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
            auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];

            // TODO: add randomness here
            addedEdges.emplace_back(nodeId1, nodeId2);
            boost::add_edge(nodeId1, nodeId2, topoGraph);

            sharedNodesList.erase(it);
            if (recursive) {
                auto sharedNodesListBackup = sharedNodesList;
                auto result = feasibilityCheckHelper(sharedNodesListBackup, recursive);
                boost::remove_edge(nodeId1, nodeId2, topoGraph);
                addedEdges.pop_back();

                if (result.first == agents.size() && result.second == agents.size()) {
                    for (auto [_nodeId1, _nodeId2]: addedEdges) {
                        boost::remove_edge(_nodeId1, _nodeId2, topoGraph);
                    }
                    return result;
                }
                auto nodeId3 = pathTopoNodeIds[it->agentId1][it->state1];
                auto nodeId4 = pathTopoNodeIds[it->agentId2][it->state2 + 1];
                addedEdges.emplace_back(nodeId4, nodeId3);
                boost::add_edge(nodeId4, nodeId3, topoGraph);
            }

//            std::cerr << "random choose: " << nodeId1 << " " << nodeId2 << std::endl;
        }
    }
    for (auto [nodeId1, nodeId2]: addedEdges) {
        boost::remove_edge(nodeId1, nodeId2, topoGraph);
    }
    return std::make_pair(agents.size(), agents.size());
}


std::pair<size_t, size_t> OnlineSimulator::feasibilityCheckTest(bool recursive) {
//    std::cerr << "start feasibility check" << std::endl;

    std::list<SharedNodePair> sharedNodesList;

    for (const auto &[nodeId, sharedNode]: sharedNodes) {
        for (const auto &sharedNodePair: sharedNode) {
            sharedNodesList.emplace_back(sharedNodePair);
        }
    }

    return feasibilityCheckHelper(sharedNodesList, recursive);

/*
    while (!sharedNodesList.empty()) {
        unsigned int erasedEdges = 0;

        for (auto it = sharedNodesList.begin(); it != sharedNodesList.end();) {
            std::vector<std::pair<unsigned int, unsigned int>> selectedEdges;
            unsigned int maxSelectedEdges = 0;
            if (it->state1 + 1 < paths[it->agentId1].size() && it->state2 > agents[it->agentId2].state) {
                auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
                auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];
                boost::add_edge(nodeId1, nodeId2, topoGraph);
                std::vector<Graph::topo_vertex_t> container;
                try {
                    boost::topological_sort(topoGraph, std::back_inserter(container));
                    selectedEdges.emplace_back(nodeId1, nodeId2);
                } catch (std::exception) {}
                boost::remove_edge(nodeId1, nodeId2, topoGraph);
                ++maxSelectedEdges;
            }
            if (it->state2 + 1 < paths[it->agentId2].size() && it->state1 > agents[it->agentId1].state) {
                auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1];
                auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2 + 1];
                boost::add_edge(nodeId2, nodeId1, topoGraph);
                try {
                    std::vector<Graph::topo_vertex_t> container;
                    boost::topological_sort(topoGraph, std::back_inserter(container));
                    selectedEdges.emplace_back(nodeId2, nodeId1);
                } catch (std::exception) {}
                boost::remove_edge(nodeId2, nodeId1, topoGraph);
                ++maxSelectedEdges;
            }
            if (selectedEdges.empty()) {
                if (maxSelectedEdges > 0) {
                    // failed
                    // std::cerr << "failed!" << std::endl;
                    for (auto [nodeId1, nodeId2]: addedEdges) {
                        boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    }
                    return std::make_pair(it->agentId1, it->agentId2);
                }
                it = sharedNodesList.erase(it);
                ++erasedEdges;
            } else if (selectedEdges.size() == 1) {
                addedEdges.emplace_back(selectedEdges[0]);
                boost::add_edge(selectedEdges[0].first, selectedEdges[0].second, topoGraph);
                it = sharedNodesList.erase(it);
                ++erasedEdges;
//                std::cerr << "choose: " << selectedEdges[0].first << " " << selectedEdges[0].second << std::endl;
            } else {
                ++it;
            }
        }

        if (erasedEdges == 0 && !sharedNodesList.empty()) {
            auto it = sharedNodesList.begin();
            auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
            auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];

            // TODO: add randomness here
            addedEdges.emplace_back(nodeId1, nodeId2);
            boost::add_edge(nodeId1, nodeId2, topoGraph);
            sharedNodesList.erase(it);
//            std::cerr << "random choose: " << nodeId1 << " " << nodeId2 << std::endl;
        }
    }
*/

//    for (auto [nodeId1, nodeId2]: addedEdges) {
//        boost::remove_edge(nodeId1, nodeId2, topoGraph);
//    }
//
//    return std::make_pair(agents.size(), agents.size());
}

std::pair<size_t, size_t> OnlineSimulator::feasibilityCheck() {
    feasibilityCheckCount++;
    auto heuristicResult = feasibilityCheckTest(true);
    auto exhaustiveResult = feasibilityCheckTest(false);
    bool heuristicSuccess = heuristicResult.first == agents.size() && heuristicResult.second == agents.size();
    bool exhaustiveSuccess = exhaustiveResult.first == agents.size() && exhaustiveResult.second == agents.size();
    if (heuristicSuccess) {
        if (exhaustiveSuccess) {
            feasibilityCheckTypes[0]++;
        } else {
            feasibilityCheckTypes[1]++;
        }
    } else {
        if (exhaustiveSuccess) {
            feasibilityCheckTypes[2]++;
        } else {
            feasibilityCheckTypes[3]++;
        }
    }
    if (isHeuristicFeasibilityCheck) {
        return heuristicResult;
    } else {
        return exhaustiveResult;
    }
}



