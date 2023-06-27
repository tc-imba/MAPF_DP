//
// Created by liuyh on 26/4/2023.
//

#include "DiscreteOnlineSimulator.h"

#include <boost/range/join.hpp>
#include <boost/graph/topological_sort.hpp>
#include <iostream>
#include <random>
#include <chrono>
#include <fstream>


unsigned int DiscreteOnlineSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                                      unsigned int delayStart, unsigned int delayInterval) {
    initSimulation();
    executionTimeVec.clear();
    openOutputFiles();

    if (outputFile.is_open()) {
        outputFile << 0 << std::endl;
        for (unsigned int i = 0; i < agents.size(); i++) {
            outputFile << i << " " << graph.getNode(paths[i][0]).index << " " << "node" << std::endl;
        }
    }

    std::uniform_real_distribution<double> distribution(0, 1);

    if (debug) {
        for (unsigned int i = 0; i < agents.size(); i++) {
//        if (agents[i].current == agents[i].goal) continue;
            std::cout << "agent " << i << "(" << agents[i].start << "->" << agents[i].goal << "): ";
            for (const auto &label: solver->solution->plans[i]->path) {
                std::cout << "(" << label.state << "," << label.nodeId << ")->";
            }
            std::cout << std::endl;
//        nodeStates[agents[i].start] = 0;
        }
    }

    executionTimeStart = std::chrono::steady_clock::now();
    for (; currentTimestep + 1 < maxTimeStep;) {

//        std::cout << currentTimestep << " " << delayStart << " " << delayInterval << std::endl;
        updateDelayedSet(currentTimestep, delayStart, delayInterval);

//        if (pauseTimestep > 0 && currentTimestep == 2) {
//            std::cout << "breakpoint" << std::endl;
//        }
//

        currentTimestep++;

        if (outputFile.is_open()) {
            outputFile << currentTimestep << std::endl;
        }
        if (debug) {
            std::cout << "begin timestep " << currentTimestep << std::endl;
        }

        auto start = std::chrono::steady_clock::now();

        /** Algorithm 2 **/

#ifdef DEBUG_CYCLE
        auto lastMoved = moved;
        auto lastBlocked = blocked;

        printSets("before init |");
        initChecks();
        printSets("after init  |");
        if (!isOnlyCycleCheck) {
            unsharedCheck();
            printSets("unshared    |");
            neighborCheck();
            printSets("neighbor    |");
            deadEndCheck();
            printSets("deadend     |");
        }

        auto savedBlocked = blocked;
        auto savedUnshared = unshared;
        auto lastReady = ready;
        cycleCheck();
        printSets("cycle       |");

        auto savedReady = ready;
        moved = lastMoved;
        blocked = lastBlocked;

        ready.clear();
        unshared.clear();
        printSets("before init |");
        initChecks();
        printSets("after init  |");
        cycleCheck();
        printSets("cycle       |");
#else
        /** line 1 **/
        initChecks();
        if (!isOnlyCycleCheck) {
            /** note that the iteration (line 2-3) occurs three times separately in the following functions **/
            /** line 4-6 **/
            unsharedCheck();
            /** line 7-8 **/
            neighborCheck();
            /** line 9-10 **/
            deadEndCheck();
        }
        /** line 12-23 **/
        cycleCheck();
#endif

        auto end = std::chrono::steady_clock::now();


#ifdef DEBUG_CYCLE
        if (ready.size() != savedReady.size() + savedUnshared.size()) {
            std::cout << currentTimestep << std::endl;
            for (unsigned int i = 0; i < agents.size(); i++) {
//        if (agents[i].current == agents[i].goal) continue;
                std::cout << "agent " << i << " " << agents[i].state << " (" << agents[i].start << "->"
                          << agents[i].goal << "): ";
                for (const auto &label: solver->solution->plans[i]->path) {
                    std::cout << "(" << label.state << "," << label.nodeId << ")->";
                }
                std::cout << std::endl;
//        nodeStates[agents[i].start] = 0;
            }
            for (const auto &pair: sharedNodes) {
                std::cout << pair.first << " ";
                for (const auto &item: pair.second) {
                    std::cout << "{" << item.agentId1 << "," << item.state1 << "," << item.agentId2 << ","
                              << item.state2 << "} ";
                }
                std::cout << std::endl;
            }

            std::cout << "    blocked ";
            for (auto i: savedBlocked) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
            std::cout << "     naive ";
            for (auto i: ready) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
            std::cout << "semi-naive ";
            for (auto i: savedReady) {
                std::cout << i << " ";
            }
            std::cout << "| ";
            for (auto i: savedUnshared) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
            exit(0);
        }
#endif

        unblocked.insert(ready.begin(), ready.end());
        unblocked.insert(unshared.begin(), unshared.end());
        ready.clear();
        unshared.clear();
//        printSets("final       |");

//        std::cout << unblocked.size() << " " << savedReady.size() + savedUnshared.size() << std::endl;

//        for (auto i: savedBlocked) {
//            if (ready.find(i) != ready.end()) {
//                exit(0);
//            }
//        }

        if (firstAgentArrivingTimestep == 0) {
            unblockedAgents += unblocked.size();
            std::chrono::duration<double> elapsed_seconds = end - start;
            executionTime += elapsed_seconds.count();
        }

//        if ((pauseTimestep == 0 && delayInterval > 0) || currentTimestep == pauseTimestep) {
//            updateDelayedSet(currentTimestep, pauseTimestep == 0);
//        }

//        if (pauseTimestep > 0 && currentTimestep == pauseTimestep) {
//            updateDelayedSet(currentTimestep);
//        }
//        updateDelayedSet(currentTimestep);

        int count = 0;
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            auto currentNodeId = paths[i][state];
            if (state + 1 >= paths[i].size()) {
                if (firstAgentArrivingTimestep == 0) {
                    firstAgentArrivingTimestep = currentTimestep;
                }
                if (debug) {
                    std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") completed" << std::endl;
                }
                agents[i].blocked = true;
                ++count;
            } else if (blocked.find(i) != blocked.end()) {
                if (debug) {
                    std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") blocked" << std::endl;
                }
                agents[i].blocked = true;
                agents[i].timestep = currentTimestep;
            } else if (unblocked.find(i) != unblocked.end()) {
//                auto &edge = graph.g[edgeId];
                agents[i].timestep = currentTimestep;
                auto nextNodeId = paths[i][state + 1];
                auto &edge = graph.getEdge(currentNodeId, nextNodeId);

                auto it = nodeAgentMap.find(nextNodeId);
                if (it != nodeAgentMap.end() && it->second != i) {
                    std::cout << i << " " << it->second << " " << nextNodeId << std::endl;
                    exit(0);
                }

                if (delayType == "agent" && delayedSet.find(i) != delayedSet.end()) {
                    if (debug) {
                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") delayed by agent"
                                  << std::endl;
                    }
                    agents[i].blocked = false;
                    nodeAgentMap[nextNodeId] = i;
                } else if (delayType == "edge" && delayedSet.find(edge.index) != delayedSet.end()) {
                    if (debug) {
                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") delayed by edge"
                                  << std::endl;
                    }
                    agents[i].blocked = false;
                    nodeAgentMap[nextNodeId] = i;
                } else {
                    auto waitingTimestep = 1;
//                auto waitingTimestep = (unsigned int) floor(10.0 * (edge.dp - 0.5) + 2);
//                auto waitingTimestep = (unsigned int) floor(1 / (1.0 - edge.dp));
//                auto waitingTimestep = (unsigned int) floor(exp(5.0 * (edge.dp - 0.5)) + 1);
/*                if (++agents[i].waitingTimestep < waitingTimestep) {
//                std::mt19937 generator(combineRandomSeed(currentNodeId, nextNodeId, currentTimestep, seed));
//                double rand = distribution(generator);
//                if (rand < edge.dp) {
                    if (debug) {
                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") delay" << std::endl;
                    }
                    nodeAgentMap[nextNodeId] = i;
                } else {*/
                    if (debug) {
                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ")->("
                                  << state + 1 << "," << nextNodeId << ")" << std::endl;
                    }
                    agents[i].blocked = true;
                    unblocked.erase(i);
                    moved.insert(i);
                    nodeAgentMap.erase(currentNodeId);
                    nodeAgentMap[nextNodeId] = i;
                    updateSharedNode(currentNodeId, i, state);
                    agents[i].current = nextNodeId;
//                    agents[i].waitingTimestep = 0;
                    state++;
                    if (state + 1 >= paths[i].size()) {
                        if (firstAgentArrivingTimestep == 0) {
                            firstAgentArrivingTimestep = currentTimestep;
                        }
                        ++count;
                    }
                }
            } else {
                if (debug) {
                    std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") error" << std::endl;
                }
                assert(0);
            }
            if (outputFile.is_open()) {
                outputFile << i << " " << graph.getNode(paths[i][state]).index << " "
                           << (agents[i].blocked ? "node" : "edge") << std::endl;
            }
        }

        if (count >= agents.size()) {
            break;
        }
        saveExecutionTime();
    }
    saveExecutionTime();

/*    bool unfinish = false;
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (agents[i].current != agents[i].goal) {
            unfinish = true;
//            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << ", goal " << agents[i].goal << std::endl;
            break;
        }
    }*/
    closeOutputFiles();

    return countCompletedAgents();
}


void DiscreteOnlineSimulator::initSharedNodes(size_t i, size_t j) {
    std::unordered_map<unsigned int, std::vector<std::pair<size_t, unsigned int> > > m;
    for (auto x: {i, j}) {
        for (size_t k = 0; k < paths[x].size(); k++) {
            m[paths[x][k]].emplace_back(x, k);
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

void DiscreteOnlineSimulator::updateSharedNode(unsigned int nodeId, size_t agentId, unsigned int state) {
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
    if (it->second.empty()) {
        sharedNodes.erase(it);
    }
}


void DiscreteOnlineSimulator::initSimulation() {
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
        agents[i].current = agents[i].start;
        agents[i].blocked = true;
        agents[i].state = 0;
        agents[i].timestep = 0;
        blocked.insert(i);
        nodeAgentMap[agents[i].current] = i;
        for (size_t j = 0; j < solver->solution->plans[i]->path.size(); j++) {
            auto newNodeId = solver->solution->plans[i]->path[j].nodeId;
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

    executionTime = 0;
    feasibilityCheckCount = 0;
    cycleCheckCount = 0;
    cycleCheckAgents = 0;
    unblockedAgents = 0;
    firstAgentArrivingTimestep = 0;
    for (int i = 0; i < 4; i++) {
        feasibilityCheckTypes[i] = 0;
    }
    feasibilityCheckUnsettledCount = 0;
    feasibilityCheckTopoCount = 0;
    feasibilityCheckLoopCount = 0;
}


void DiscreteOnlineSimulator::initChecks() {
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

void DiscreteOnlineSimulator::unsharedCheck() {
    for (auto it = ready.begin(); it != ready.end();) {
        auto &agent = agents[*it];
        auto nextNodeId = paths[*it][agent.state + 1];
        auto it2 = sharedNodes.find(nextNodeId);
        if (it2 == sharedNodes.end() || it2->second.empty()) {
            unshared.insert(*it);
            it = ready.erase(it);
        } else {
            ++it;
        }
    }
}


void DiscreteOnlineSimulator::neighborCheck() {
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

void DiscreteOnlineSimulator::deadEndCheck() {
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

void DiscreteOnlineSimulator::naiveCycleCheckHelper(std::vector<size_t> &readyList, size_t length, size_t start,
                                                    size_t current,
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
//        std::cout << "naive cycle check: ";
//        for (auto i: checkedReady) {
//            std::cout << i << " ";
//        }
//        std::cout << std::endl;
        if (checkedReady.size() <= maxReadyList.size()) return;
        for (auto i: checkedReady) {
            agents[i].state++;
        }
        // perform a neighbor check here
        auto agentId1 = agents.size(), agentId2 = agents.size();
        for (size_t i = 0; i < agents.size(); i++) {
            auto nodeId = paths[i][agents[i].state];
            auto it = nodeAgentMap.find(nodeId);
            if (it != nodeAgentMap.end() && it->second != i) {
//                std::cout << "neighbor in cycle: " << i << " " << nodeId << " " << it->second << std::endl;
                agentId1 = i;
                agentId2 = it->second;
                break;
            }
        }
        if (agentId1 == agents.size() && agentId2 == agents.size()) {
            auto pair = feasibilityCheck();
            agentId1 = pair.first;
            agentId2 = pair.second;
        }
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

void DiscreteOnlineSimulator::naiveCycleCheck() {
//    std::cerr << ready.size() << std::endl;
    std::vector<size_t> readyList;
    readyList.insert(readyList.begin(), ready.begin(), ready.end());
    std::vector<bool> check(readyList.size(), false);
    std::vector<size_t> maxReadyList;
    for (size_t i = 1; i <= readyList.size(); i++) {
        naiveCycleCheckHelper(readyList, i, 0, 0, check, maxReadyList);
    }
    std::set<size_t> newReady(maxReadyList.begin(), maxReadyList.end());
    for (auto i: ready) {
        if (newReady.find(i) == newReady.end()) {
            blocked.insert(i);
        }
    }
    ready.swap(newReady);
//    ready.clear();
//    ready.insert(maxReadyList.begin(), maxReadyList.end());
}

/** Algorithm 2: line 12-23 **/
void DiscreteOnlineSimulator::heuristicCycleCheck() {
    //    std::cerr << "start cycle check: ";
    std::set<size_t> newReady = ready;
    for (auto i: newReady) {
        agents[i].state++;
//        std::cerr << i << " ";
    }
//    std::cerr << std::endl;

    // use feasibility check to block some agents
    std::set<size_t> removed;
    /** line 12 **/
    while (!newReady.empty()) {
//        std::cerr << "check 1" << std::endl;
        /** line 13 is done before the iteration and updated in each iteration **/
        /** line 14 **/
        auto [agentId1, agentId2] = feasibilityCheck();
        /** line 15 **/
        if (agentId1 == agents.size() && agentId2 == agents.size()) {
            break;
        }
        /** line 16-17 **/
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
    /** line 18-19 **/
    for (auto i: ready) {
        if (newReady.find(i) == newReady.end()) {
            agents[i].state++;
//            std::cerr << "check 2: " << i << std::endl;
            /** line 20 **/
            auto [agentId1, agentId2] = feasibilityCheck();
            /** line 21-23 **/
            if (agentId1 == agents.size() && agentId2 == agents.size()) {
                newReady.insert(i);
                // we find that whether to break here does not affect the result
                // you can comment out the next line to achieve line 23 in algorithm 2
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

    for (auto i: ready) {
        if (newReady.find(i) == newReady.end()) {
            blocked.insert(i);
        }
    }
    ready.swap(newReady);
}


void DiscreteOnlineSimulator::cycleCheck() {
    if (ready.empty()) return;
    if (firstAgentArrivingTimestep == 0) {
        cycleCheckCount++;
        cycleCheckAgents += ready.size();
    }
    // suppose all unblocked agents are at their next state
    for (auto i: unblocked) {
        agents[i].state++;
    }
    for (auto i: unshared) {
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
    for (auto i: unshared) {
        agents[i].state--;
    }
//    std::cerr << "end cycle check: ";
}

bool DiscreteOnlineSimulator::isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
    auto visitor = TopoGraphBFSVisitor(nodeId2);
    try {
        boost::breadth_first_search(topoGraph, nodeId1, boost::visitor(visitor));
    } catch (...) {
        return true;
    }
    return false;
}

/** Algorithm 1 **/
std::pair<size_t, size_t> DiscreteOnlineSimulator::feasibilityCheckHelper(
        std::list<SharedNodePair> &sharedNodesList,
        bool recursive
) {
    std::vector<std::pair<unsigned int, unsigned int>> addedEdges;
//    std::cout << "feasibility check" << std::endl;

    /** line 1 **/
    while (!sharedNodesList.empty()) {
        /** line 2 **/
        if (firstAgentArrivingTimestep == 0) {
            feasibilityCheckLoopCount++;
        }
        unsigned int erasedEdges = 0;
        /** line 3 **/


        for (auto it = sharedNodesList.begin(); it != sharedNodesList.end();) {
//            feasibilityCheckIterationTemp++;
            if (firstAgentArrivingTimestep == 0) {
                feasibilityCheckTopoCount++;
            }
            std::vector<std::pair<unsigned int, unsigned int>> selectedEdges;
            unsigned int maxSelectedEdges = 0;
            if (it->state2 > agents[it->agentId2].state) {
                if (it->state1 + 1 == paths[it->agentId1].size()) {
                    ++maxSelectedEdges;
                } else if (it->state1 + 1 < paths[it->agentId1].size() && it->state2 > agents[it->agentId2].state) {
                    auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
                    auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];
                    if (!isPathInTopoGraph(nodeId2, nodeId1)) {
                        selectedEdges.emplace_back(nodeId1, nodeId2);
                    }
//                    boost::add_edge(nodeId1, nodeId2, topoGraph);
//                    std::vector<Graph::topo_vertex_t> container;
//                    try {
//                        boost::topological_sort(topoGraph, std::back_inserter(container));
//                        selectedEdges.emplace_back(nodeId1, nodeId2);
//                    } catch (std::exception) {}
//                    boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    ++maxSelectedEdges;
                }
            }
            if (it->state1 > agents[it->agentId1].state) {
                if (it->state2 + 1 == paths[it->agentId2].size()) {
                    ++maxSelectedEdges;
                } else if (it->state2 + 1 < paths[it->agentId2].size() && it->state1 > agents[it->agentId1].state) {
                    auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1];
                    auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2 + 1];
                    if (!isPathInTopoGraph(nodeId1, nodeId2)) {
                        selectedEdges.emplace_back(nodeId2, nodeId1);
                    }
//                    boost::add_edge(nodeId2, nodeId1, topoGraph);
//                    try {
//                        std::vector<Graph::topo_vertex_t> container;
//                        boost::topological_sort(topoGraph, std::back_inserter(container));
//                        selectedEdges.emplace_back(nodeId2, nodeId1);
//                    } catch (std::exception) {}
//                    boost::remove_edge(nodeId2, nodeId1, topoGraph);
                    ++maxSelectedEdges;
                }
            }

            if (selectedEdges.empty()) {
                /** line 4 **/
                if (maxSelectedEdges > 0) {
                    // failed
//                    std::cout << "failed: " << it->agentId1 << " " << it->state1 << " " << it->agentId2 << ""
//                              << it->state2 << std::endl;
                    for (auto [nodeId1, nodeId2]: addedEdges) {
                        boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    }
//                    std::cout << "infeasible" << std::endl;
                    /** line 5 **/
                    return std::make_pair(it->agentId1, it->agentId2);
                } else {
//                    std::cout << "failed: " << it->agentId1 << " " << it->state1 << " " << it->agentId2 << ""
//                              << it->state2 << std::endl;
                }
                /** unreachable code, only for execution safety  **/
                it = sharedNodesList.erase(it);
                ++erasedEdges;
                /** unreachable code end **/
            } /** line 6 **/
            else if (selectedEdges.size() == 1) {
                /** line 7 **/
                addedEdges.emplace_back(selectedEdges[0]);
                boost::add_edge(selectedEdges[0].first, selectedEdges[0].second, topoGraph);
                /** line 8 **/
                it = sharedNodesList.erase(it);
                /** line 9 **/
                ++erasedEdges;
//                std::cout << "choose: " << selectedEdges[0].first << " " << selectedEdges[0].second << std::endl;
            } else {
                ++it;
            }
        }

//        std::cout << erasedEdges << std::endl;

        /** line 10 **/
        if (erasedEdges == 0 && !sharedNodesList.empty()) {
            /** line 11 **/
            auto it = sharedNodesList.begin();
            auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
            auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];

            /** line 12 **/
            // TODO: add randomness here
            addedEdges.emplace_back(nodeId1, nodeId2);
            boost::add_edge(nodeId1, nodeId2, topoGraph);

            /** line 13 **/
            sharedNodesList.erase(it);

            /** exhaustive version of Algorithm 1 **/
            if (recursive) {
                auto sharedNodesListBackup = sharedNodesList;
//                std::cout << "recursive" << std::endl;
                auto result = feasibilityCheckHelper(sharedNodesListBackup, recursive);
                if (firstAgentArrivingTimestep > 0) {
                    feasibilityCheckRecursionCount++;
                }
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
//            std::cout << "random choose: " << nodeId1 << " " << nodeId2 << std::endl;
        }
    }
    for (auto [nodeId1, nodeId2]: addedEdges) {
        boost::remove_edge(nodeId1, nodeId2, topoGraph);
    }
//    std::cout << "feasible" << std::endl;
    /** line 14 **/
    return std::make_pair(agents.size(), agents.size());
}


std::pair<size_t, size_t> DiscreteOnlineSimulator::feasibilityCheckTest(bool recursive) {
//    std::cerr << "start feasibility check" << std::endl;

    std::list<SharedNodePair> sharedNodesList;

    std::unordered_map<size_t, size_t> nodeAgentMapSnapshot;
    for (size_t i = 0; i < agents.size(); i++) {
        auto nodeId = paths[i][agents[i].state];
        auto it = nodeAgentMapSnapshot.find(nodeId);
        if (it == nodeAgentMapSnapshot.end()) {
            nodeAgentMapSnapshot.emplace_hint(it, nodeId, i);
        } else {
            return std::make_pair(it->second, i);
        }
    }

    for (const auto &[nodeId, sharedNode]: sharedNodes) {
        for (const auto &sharedNodePair: sharedNode) {
            sharedNodesList.emplace_back(sharedNodePair);
        }
    }

    if (firstAgentArrivingTimestep == 0) {
        feasibilityCheckUnsettledCount += sharedNodesList.size();
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

std::pair<size_t, size_t> DiscreteOnlineSimulator::feasibilityCheck() {
    if (firstAgentArrivingTimestep == 0) {
        feasibilityCheckCount++;
    }
    if (!isFeasibilityType) {
        return feasibilityCheckTest(!isHeuristicFeasibilityCheck);
    }
//    feasibilityCheckIterationTemp = 0;
    auto heuristicResult = feasibilityCheckTest(false);
//    feasibilityCheckIteration[0] += feasibilityCheckIterationTemp;
//    feasibilityCheckIterationTemp = 0;
    auto exhaustiveResult = feasibilityCheckTest(true);
//    feasibilityCheckIteration[1] += feasibilityCheckIterationTemp;
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