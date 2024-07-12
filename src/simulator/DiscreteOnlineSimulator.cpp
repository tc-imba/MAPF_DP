//
// Created by liuyh on 26/4/2023.
//

#include "DiscreteOnlineSimulator.h"

#include <boost/graph/topological_sort.hpp>
#include <boost/range/join.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>


unsigned int DiscreteOnlineSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                                               unsigned int delayStart, unsigned int delayInterval) {
    initSimulation();
//    executionTimeVec.clear();
//    agentCountVec.clear();
    openOutputFiles();
    std::vector<std::string> agentOutputStates(agents.size());

    if (outputFile.is_open()) {
        outputFile << 0 << std::endl;
        for (unsigned int i = 0; i < agents.size(); i++) {
            outputFile << i << " " << graph.getNode(paths[i][0]).index << " "
                       << "node"
                       << " "
                       << "block" << std::endl;
        }
    }

    std::uniform_real_distribution<double> distribution(0, 1);

    if (debug) {
        std::ostringstream oss;
        for (unsigned int i = 0; i < agents.size(); i++) {
            //        if (agents[i].current == agents[i].goal) continue;
            oss.str("");
            oss.clear();
            oss << "agent " << i << " (" << agents[i].start << "->" << agents[i].goal << "): ";
            for (unsigned int j = 0; j < paths[i].size(); j++) { oss << "(" << j << "," << paths[i][j] << ")->"; }
            SPDLOG_DEBUG("{}", oss.str());
            //        nodeStates[agents[i].start] = 0;
        }
    }

    // warmup
    //    depGraph.feasibilityCheckTest(false);

    for (; currentTimestep + 1 < maxTimeStep;) {
        if (verboseOutput) {
            timestepJson.clear();
            timestepJson["timestep"] = currentTimestep;
            executionTimeStart = std::chrono::steady_clock::now();
        }

        //        std::cout << currentTimestep << " " << delayStart << " " << delayInterval << std::endl;
        updateDelayedSet(currentTimestep, delayStart, delayInterval);

        //        if (pauseTimestep > 0 && currentTimestep == 2) {
        //            std::cout << "breakpoint" << std::endl;
        //        }
        //

        currentTimestep++;

        if (outputFile.is_open()) { outputFile << currentTimestep << std::endl; }
        SPDLOG_DEBUG("begin timestep {}", currentTimestep);

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

        printSets("before init |", currentTimestep);
        /** line 1 **/
        initChecks();
        printSets("after init  |", currentTimestep);
        if (!isOnlyCycleCheck) {
            /** note that the iteration (line 2-3) occurs three times separately in the following functions **/
            /** line 4-6 **/
            unsharedCheck();
            printSets("unshared    |", currentTimestep);
            /** line 7-8 **/
            neighborCheck();
            printSets("neighbor    |", currentTimestep);
            /** line 9-10 **/
            deadEndCheck();
            printSets("deadend     |", currentTimestep);
        }
        /** line 12-23 **/

        decltype(start) end;
        if (currentTimestep == 1) { end = std::chrono::steady_clock::now(); }

        cycleCheck();
        printSets("cycle       |", currentTimestep);
#endif

        if (currentTimestep > 1) { end = std::chrono::steady_clock::now(); }


        //        auto end = std::chrono::steady_clock::now();


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
            for (auto i: savedBlocked) { std::cout << i << " "; }
            std::cout << std::endl;
            std::cout << "     naive ";
            for (auto i: ready) { std::cout << i << " "; }
            std::cout << std::endl;
            std::cout << "semi-naive ";
            for (auto i: savedReady) { std::cout << i << " "; }
            std::cout << "| ";
            for (auto i: savedUnshared) { std::cout << i << " "; }
            std::cout << std::endl;
            exit(0);
        }
#endif

        unblocked.insert(ready.begin(), ready.end());
        unblocked.insert(unshared.begin(), unshared.end());
        ready.clear();
        unshared.clear();
        printSets("final       |", currentTimestep);
        if (verboseOutput) {
            timestepJson["blockedAgents"] = blocked.size();
            timestepJson["unblockedAgents"] = unblocked.size();
        }

//        agentCountVec.emplace_back(blocked.size(), unblocked.size());


        //        std::cout << unblocked.size() << " " << savedReady.size() + savedUnshared.size() << std::endl;

        //        for (auto i: savedBlocked) {
        //            if (ready.find(i) != ready.end()) {
        //                exit(0);
        //            }
        //        }

        std::chrono::duration<double> elapsed_seconds = end - start;
        //        if (firstAgentArrivingTimestep == 0) {
        unblockedAgents += unblocked.size();
        //        }
        if (firstAgentArrivingTimestep == 0) { firstAgentArrivingExecutionTime += elapsed_seconds.count(); }
        executionTime += elapsed_seconds.count();

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
                if (firstAgentArrivingTimestep == 0) { firstAgentArrivingTimestep = currentTimestep; }
                SPDLOG_DEBUG("agent {}: ({},{}) {}", i, state, currentNodeId, "completed");
                agentOutputStates[i] = "complete";
                agents[i].blocked = true;
                ++count;
            } else if (blocked.find(i) != blocked.end()) {
                SPDLOG_DEBUG("agent {}: ({},{}) {}", i, state, currentNodeId, "blocked");
                agentOutputStates[i] = "block";
                agents[i].blocked = true;
                agents[i].timestep = currentTimestep;
            } else if (unblocked.find(i) != unblocked.end()) {
                //                auto &edge = graph.g[edgeId];
                agents[i].timestep = currentTimestep;
                auto nextNodeId = paths[i][state + 1];
                auto &edge = graph.getEdge(currentNodeId, nextNodeId);

                auto it = nodeAgentMap.find(nextNodeId);
                if (it != nodeAgentMap.end() && it->second != i) {
                    SPDLOG_ERROR("agent {}: ({},{})->({},{}) conflict", i, state, currentNodeId, state + 1, nextNodeId);
                    std::cout << "agent " << i << " and " << it->second << " conflict at " << nextNodeId << std::endl;
                    exit(0);
                }

                if (delayType == "agent" && delayedSet.find(i) != delayedSet.end()) {
                    SPDLOG_DEBUG("agent {}: ({},{}) {}", i, state, currentNodeId, "delayed by agent");
                    agentOutputStates[i] = "delay";
                    agents[i].blocked = false;
                    nodeAgentMap[nextNodeId] = i;
                } else if (delayType == "edge" && delayedSet.find(edge.index) != delayedSet.end()) {
                    SPDLOG_DEBUG("agent {}: ({},{}) {}", i, state, currentNodeId, "delayed by edge");
                    agentOutputStates[i] = "delay";
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
                    SPDLOG_DEBUG("agent {}: ({},{})->({},{})", i, state, currentNodeId, state + 1, nextNodeId);
                    agentOutputStates[i] = "move";
                    agents[i].blocked = true;
                    unblocked.erase(i);
                    moved.insert(i);
                    nodeAgentMap.erase(currentNodeId);
                    nodeAgentMap[nextNodeId] = i;
                    depGraph.updateSharedNode(currentNodeId, i, state);
                    agents[i].current = nextNodeId;
                    //                    agents[i].waitingTimestep = 0;
                    state++;
                    if (state + 1 >= paths[i].size()) {
                        if (firstAgentArrivingTimestep == 0) { firstAgentArrivingTimestep = currentTimestep; }
                        ++count;
                    }
                }
            } else {
                SPDLOG_ERROR("agent {}: ({},{}) {}", i, state, currentNodeId, "error");
                assert(0);
            }
            if (outputFile.is_open()) {
                auto &node = graph.getNode(paths[i][state]);
                auto delayed = delayedSet.find(i) != delayedSet.end();
                outputFile << i << " " << node.index << " " << (agents[i].blocked ? "node" : "edge") << " "
                           << agentOutputStates[i] << std::endl;
            }
        }

        if (count >= agents.size()) {break; }
        if (verboseOutput) {
            writeTimestepOutput();
        }
    }
    if (verboseOutput) {
        writeTimestepOutput();
    }

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


void DiscreteOnlineSimulator::initSimulation() {
    blocked.clear();
    paths.clear();
    paths.resize(agents.size());
    //    deadEndStates.clear();
    //    deadEndStates.resize(agents.size());
    //    pathTopoNodeIds.clear();
    //    pathTopoNodeIds.resize(agents.size());
    nodeAgentMap.clear();
    delayedSet.clear();
    //    topoGraph.clear();

    //    unsigned int topoGraphNodeNum = 0;
    for (size_t i = 0; i < agents.size(); i++) {
        agents[i].current = agents[i].start;
        agents[i].blocked = true;
        agents[i].state = 0;
        agents[i].timestep = 0;
        blocked.insert(i);
        nodeAgentMap[agents[i].current] = i;
        /*for (size_t j = 0; j < solver->solution->plans[i]->path.size(); j++) {
            auto newNodeId = solver->solution->plans[i]->path[j].nodeId;
            if (paths[i].empty() || paths[i].back() != newNodeId) {
                paths[i].emplace_back(newNodeId);
                pathTopoNodeIds[i].emplace_back(boost::add_vertex(topoGraph));
            }
        }
        for (size_t j = 0; j < paths[i].size() - 1; j++) {
            boost::add_edge(pathTopoNodeIds[i][j], pathTopoNodeIds[i][j + 1], topoGraph);
        }
        topoGraphNodeNum += paths[i].size();*/
    }


    /*    for (size_t i = 0; i < agents.size(); i++) {
        for (size_t j = i + 1; j < agents.size(); j++) {
            initSharedNodes(i, j);
        }
    }*/

    firstAgentArrivingExecutionTime = 0;
    executionTime = 0;
    feasibilityCheckCount = 0;
    cycleCheckCount = 0;
    cycleCheckAgents = 0;
    unblockedAgents = 0;
    firstAgentArrivingTimestep = 0;
    for (int i = 0; i < 4; i++) { feasibilityCheckTypes[i] = 0; }
    feasibilityCheckUnsettledCount = 0;
    feasibilityCheckTopoCount = 0;
    feasibilityCheckLoopCount = 0;

    depGraph.solver = solver;
    depGraph.debug = debug;
    depGraph.onlineOpt = onlineOpt;
    depGraph.groupDetermined = groupDetermined;
    depGraph.isFastCycleCheck = isFastCycleCheck;
    depGraph.init();
}


void DiscreteOnlineSimulator::initChecks() {
    ready.clear();
    for (auto i: boost::join(blocked, moved)) {
        auto &agent = agents[i];
        if (agent.state + 1 < paths[i].size()) { ready.insert(i); }
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
        auto it2 = depGraph.sharedNodes.find(nextNodeId);
        if (it2 == depGraph.sharedNodes.end() || it2->second.empty()) {
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
            for (auto &&[j, state]: depGraph.deadEndStates[*it]) {
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
                                                    size_t current, std::vector<bool> &check,
                                                    std::vector<size_t> &maxReadyList) {
    if (length <= maxReadyList.size()) return;
    if (current > length) return;
    if (current == length) {
        std::vector<size_t> checkedReady;
        for (size_t i = 0; i < readyList.size(); i++) {
            if (check[i]) { checkedReady.emplace_back(readyList[i]); }
        }
        //        std::cout << "naive cycle check: ";
        //        for (auto i: checkedReady) {
        //            std::cout << i << " ";
        //        }
        //        std::cout << std::endl;
        if (checkedReady.size() <= maxReadyList.size()) return;
        for (auto i: checkedReady) { agents[i].state++; }
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
        for (auto i: checkedReady) { agents[i].state--; }
        if (agentId1 == agents.size() && agentId2 == agents.size()) { maxReadyList.swap(checkedReady); }
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
    for (size_t i = 1; i <= readyList.size(); i++) { naiveCycleCheckHelper(readyList, i, 0, 0, check, maxReadyList); }
    std::set<size_t> newReady(maxReadyList.begin(), maxReadyList.end());
    for (auto i: ready) {
        if (newReady.find(i) == newReady.end()) { blocked.insert(i); }
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
        if (agentId1 == agents.size() && agentId2 == agents.size()) { break; }
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
        if (newReady.find(i) == newReady.end()) { blocked.insert(i); }
    }
    ready.swap(newReady);
}

bool DiscreteOnlineSimulator::fastCycleCheck() {
    std::vector<bool> canAdd(agents.size(), false);
    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].newState = agents[i].state;
    }
    for (auto i: ready) {
        agents[i].newState++;
        canAdd[i] = true;
    }
    /*
    // prevent collision at next node
    std::unordered_map<size_t, size_t> nodeAgentCurrentMap;
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (!canAdd[i]) {
            agents[i].newState = agents[i].state;
            auto nodeId = paths[i][agents[i].newState];
            nodeAgentCurrentMap.emplace(nodeId, i);
        }
    }
    for (auto i: ready) {
        auto nodeId = paths[i][agents[i].newState];
        auto it = nodeAgentCurrentMap.find(nodeId);
        if (it == nodeAgentCurrentMap.end()) {
            nodeAgentCurrentMap.emplace_hint(it, nodeId, i);
        } else {
            canAdd[i] = false;
            agents[i].newState = agents[i].state;
        }
    }*/

    auto result = feasibilityCheck(true);
    if (result.first != agents.size() || result.second != agents.size()) {
        SPDLOG_WARN("initial plan not feasible!");
        std::cerr << "warning: initial plan not feasible!" << std::endl;
        if (depGraph.savedAddedEdges.empty()) {
            result = feasibilityCheck(false);
            //        exit(-1);
            if (result.first != agents.size() || result.second != agents.size()) {
                return false;
            }
        }
        /*for (auto i: ready) { blocked.insert(i); }
        ready.clear();
        if (unblocked.empty() && unshared.empty()) {
            SPDLOG_ERROR("ready and unblocked both empty!");
            std::cerr << "error: ready and unblocked both empty!" << std::endl;
            exit(-1);
        }
        return;*/
    }

    //    for (auto i: ready) {
    //        agents[i].state++;
    //        canAdd[i] = true;
    //    }
    for (const auto &[edgePair, edge]: depGraph.savedAddedEdges) {
//        SPDLOG_DEBUG("savedAddedEdges: {} {} -> {}", edgePair.first,
//                     edgePair.second, edge);
        auto agentId = edge.dest.agentId;
        auto state = edge.dest.state;
        if (agents[edge.source.agentId].state >= edge.source.state) {
            continue;
        }
        if (canAdd[agentId] && agents[agentId].newState >= state) {
            SPDLOG_DEBUG("unsettled edge conflict on agent {} (saved) : {} {} -> {}", agentId, edgePair.first,
                         edgePair.second, edge);
            canAdd[agentId] = false;
        }
    }
    for (const auto &[edgePair, edge]: depGraph.ignoredEdges) {
        auto agentId = edge.dest.agentId;
        auto state = edge.dest.state;
        if (agents[edge.source.agentId].state >= edge.source.state) {
            continue;
        }
        if (canAdd[agentId] && agents[agentId].newState >= state) {
            SPDLOG_DEBUG("unsettled edge conflict on agent {} (ignored): {} {} -> {}", agentId, edgePair.first,
                         edgePair.second, edge);
            canAdd[agentId] = false;
        }
    }
    //    for (auto i: ready) {
    //        agents[i].state--;
    //    }
    std::set<size_t> newReady;
    for (auto i: ready) {
        if (!canAdd[i]) {
            blocked.insert(i);
        } else {
            newReady.emplace(i);
        }
    }
    ready.swap(newReady);
    return true;
}


void DiscreteOnlineSimulator::cycleCheck() {
//    depGraph.feasibilityCheckUnsettledEdgePairsCount = 0;
    depGraph.feasibilityCheckLoopCount = 0;
    depGraph.feasibilityCheckRecursionCount = 0;
    depGraph.feasibilityCheckTopoCount = 0;
    feasibilityCheckCount = 0;

    if (ready.empty()) return;
    //    if (firstAgentArrivingTimestep == 0) {
    cycleCheckCount++;
    cycleCheckAgents += ready.size();
    //    }
    // suppose all unblocked agents are at their next state
    for (auto i: unblocked) { agents[i].state++; }
    for (auto i: unshared) { agents[i].state++; }

    bool checkFinished = false;
    if (isFastCycleCheck) {
        checkFinished = fastCycleCheck();
    }
    if (!checkFinished) {
        if (isHeuristicCycleCheck) {
            heuristicCycleCheck();
        } else {
            naiveCycleCheck();
        }
    }


    // recover the states of the agents
    for (auto i: unblocked) { agents[i].state--; }
    for (auto i: unshared) { agents[i].state--; }
    //    std::cerr << "end cycle check: ";

    if (verboseOutput) {
        timestepJson["feasibilityCheckCount"] = feasibilityCheckCount;
        timestepJson["feasibilityCheckUnsettledEdgePairsCount"] = depGraph.feasibilityCheckUnsettledEdgePairsCount;
        timestepJson["feasibilityCheckLoopCount"] = depGraph.feasibilityCheckLoopCount;
        timestepJson["feasibilityCheckRecursionCount"] = depGraph.feasibilityCheckRecursionCount;
        timestepJson["feasibilityCheckTopoCount"] = depGraph.feasibilityCheckTopoCount;

    }

}


std::pair<size_t, size_t> DiscreteOnlineSimulator::feasibilityCheck(bool fast) {
    //    if (firstAgentArrivingTimestep == 0) {
    feasibilityCheckCount++;
    //    }
    return depGraph.feasibilityCheckTest(!isHeuristicFeasibilityCheck, true, fast);

    /*if (!isFeasibilityType) { return depGraph.feasibilityCheckTest(!isHeuristicFeasibilityCheck, true, fast); }
    //    feasibilityCheckIterationTemp = 0;
    auto heuristicResult = depGraph.feasibilityCheckTest(false, true, fast);
    //    feasibilityCheckIteration[0] += feasibilityCheckIterationTemp;
    //    feasibilityCheckIterationTemp = 0;
    auto exhaustiveResult = depGraph.feasibilityCheckTest(false, true, fast);
    //    feasibilityCheckIteration[1] += feasibilityCheckIterationTemp;
    bool heuristicSuccess = heuristicResult.first == agents.size() && heuristicResult.second == agents.size();
    bool exhaustiveSuccess = exhaustiveResult.first == agents.size() && exhaustiveResult.second == agents.size();
    if (heuristicSuccess) {
        if (exhaustiveSuccess) {
            feasibilityCheckTypes[0]++;
        } else {
            feasibilityCheckTypes[1]++;
            exit(0);
        }
    } else {
        if (exhaustiveSuccess) {
            feasibilityCheckTypes[2]++;
            exit(0);
        } else {
            feasibilityCheckTypes[3]++;
        }
    }
    return heuristicResult;
    //    if (isHeuristicFeasibilityCheck) {
    //        return heuristicResult;
    //    } else {
    //        return exhaustiveResult;
    //    }*/
}