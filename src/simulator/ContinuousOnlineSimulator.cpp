//
// Created by liu on 7/1/2022.
//

#include <boost/range/join.hpp>
#include <boost/graph/topological_sort.hpp>
#include <iostream>
#include <random>
#include <chrono>
#include <fstream>

#include "ContinuousOnlineSimulator.h"


unsigned int ContinuousOnlineSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                                                 unsigned int delayStart, unsigned int delayInterval) {
    initSimulation();
    executionTimeVec.clear();
    openOutputFiles();

    // check whether the initial plan is feasible
    auto [agentId1, agentId2] = feasibilityCheck();
    if (agentId1 != agents.size() || agentId2 != agents.size()) {
        std::cerr << "initial plan is not feasible" << std::endl;
        exit(-1);
    }


    if (outputFile.is_open()) {
        outputFile << 0 << std::endl;
        for (unsigned int i = 0; i < agents.size(); i++) {
            outputFile << i << " " << graph.getNode(paths[i][0]).index << std::endl;
        }
    }

//    std::uniform_real_distribution<double> distribution(0, 1);

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

    while (currentTimestep < maxTimeStep) {
        if (outputFile.is_open()) {
            outputFile << currentTimestep << std::endl;
        }
        if (debug) {
            std::cout << "begin timestep " << currentTimestep << std::endl;
        }
//        std::cout << currentTimestep << " " << delayStart << " " << delayInterval << std::endl;
/*        if (currentTimestep >= delayStart && (size_t) (currentTimestep - delayStart) % delayInterval == 0) {
            if (debug) {
                std::cout << "update delayed set" << std::endl;
            }
            updateDelayedSet(currentTimestep);
//            delayedSet.clear();
        }*/

//        if (pauseTimestep > 0 && currentTimestep == 2) {
//            std::cout << "breakpoint" << std::endl;
//        }
//
        auto start = std::chrono::steady_clock::now();

        // agent arriving around current timestep
        moved.clear();
        double newCurrentTimestep = currentTimestep;
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            if (state + 1 < pathTopoNodeIds[i].size() && state % 2 == 1 &&
                agents[i].arrivingTimestep <= currentTimestep) {

                newCurrentTimestep = std::max(newCurrentTimestep, agents[i].arrivingTimestep);
                auto currentNodeId = paths[i][state / 2];
                auto nextNodeId = paths[i][state / 2 + 1];

                if (outputFile.is_open()) {
                    outputFile << i << " " << graph.getNode(nextNodeId).index << std::endl;
                }
                if (debug) {
                    printAgent(i, "");
                }

                unblocked.erase(i);
                moved.insert(i);
                nodeAgentMap.erase(currentNodeId);
                agents[i].current = nextNodeId;
                agents[i].timestep = agents[i].arrivingTimestep;
                ++state;
                updateSharedNode(i, state);
                if (state + 1 >= pathTopoNodeIds[i].size()) {
                    if (firstAgentArrivingTimestep == 0) {
                        firstAgentArrivingTimestep = agents[i].arrivingTimestep;
                    }
                }

            }
        }
        currentTimestep = newCurrentTimestep;

        if (debug) {
            std::cout << "expanded timestep " << currentTimestep << std::endl;
        }
//        std::cout << "timestep " << currentTimestep << std::endl;

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
//        if (!isOnlyCycleCheck) {
//            /** note that the iteration (line 2-3) occurs three times separately in the following functions **/
//            /** line 4-6 **/
//            unsharedCheck();
//            /** line 7-8 **/
//            neighborCheck();
//            /** line 9-10 **/
//            deadEndCheck();
//        }
        /** line 12-23 **/
        printSets("init        |");
        singleAgentCheck();
        printSets("cycle       |");
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

/*        for (auto i: keepMoving) {
            if (unshared.find(i) == unshared.end()) {
                std::cout << i << " in keepMoving not in unshared" << std::endl;
            }
            if (unblocked.find(i) == unblocked.end()) {
                std::cout << i << " in keepMoving not in unblocked" << std::endl;
            }
        }*/

        ready.clear();
        unshared.clear();
        printSets("final       |");

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
            auto currentNodeId = paths[i][state / 2];
//            if (debug) {
//                std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") initial" << std::endl;
//            }

            if (state + 1 >= pathTopoNodeIds[i].size()) {
                if (firstAgentArrivingTimestep == 0) {
                    firstAgentArrivingTimestep = currentTimestep;
                }
                if (debug) {
                    printAgent(i, "completed");
//                    std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") completed" << std::endl;
                }
                ++count;
                continue;
            }

            if (state % 2 == 0) {
                // node state
                if (blocked.find(i) != blocked.end()) {
                    if (debug) {
                        printAgent(i, "blocked");
//                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") blocked"
//                                  << std::endl;
                    }
                    agents[i].timestep = currentTimestep;
                    continue;
                }
                if (unblocked.find(i) != unblocked.end()) {
//                auto &edge = graph.g[edgeId];
                    agents[i].timestep = currentTimestep;

                    auto nextNodeId = paths[i][state / 2 + 1];
                    auto &edge = graph.getEdge(currentNodeId, nextNodeId);

                    auto it = nodeAgentMap.find(nextNodeId);
                    if (it != nodeAgentMap.end() && it->second != i) {
                        std::cerr << "error: collision " << i << " " << it->second << " " << nextNodeId << std::endl;
                        exit(-1);
                    }

                    agents[i].arrivingTimestep = getAgentArrivingTime(currentTimestep, delayInterval, i, edge);
                    if (debug) {
                        printAgent(i, "");
//                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ")->("
//                                  << state + 1 << "," << currentNodeId << "-" << nextNodeId << ")" << std::endl;
                    }
                    ++state;
                    nodeAgentMap[nextNodeId] = i;
                    updateSharedNode(i, state);
                    arrivingTimestepSet.insert(agents[i].arrivingTimestep);
                }
            }

            if (state % 2 == 1) {
                // edge state (only a check now)

                if (unblocked.find(i) == unblocked.end()) {
                    std::cerr << "error: agent " << i << " is in edge state " << state << " but also blocked"
                              << std::endl;
                    exit(-1);
                }

                if (debug) {
                    printAgent(i, "unblocked");
                }
/*                auto nextNodeId = paths[i][state / 2 + 1];
                auto &edge = graph.getEdge(currentNodeId, nextNodeId);

                if (delayType == "agent" && delayedSet.find(i) != delayedSet.end()) {
                    if (debug) {
                        printAgent(i, "delayed by agent");
//                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") delayed by agent"
//                                  << std::endl;
                    }
//                    nodeAgentMap[nextNodeId] = i;
                    continue;
                }
                if (delayType == "edge" && delayedSet.find(edge.index) != delayedSet.end()) {
                    if (debug) {
                        printAgent(i, "delayed by edge");
//                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") delayed by edge"
//                                  << std::endl;
                    }
//                    nodeAgentMap[nextNodeId] = i;
                    continue;
                }

                auto waitingTimestep = 1;
//                auto waitingTimestep = (unsigned int) floor(10.0 * (edge.dp - 0.5) + 2);
//                auto waitingTimestep = (unsigned int) floor(1 / (1.0 - edge.dp));
//                auto waitingTimestep = (unsigned int) floor(exp(5.0 * (edge.dp - 0.5)) + 1);
                if (++agents[i].waitingTimestep < waitingTimestep) {
//                std::mt19937 generator(combineRandomSeed(currentNodeId, nextNodeId, currentTimestep, seed));
//                double rand = distribution(generator);
//                if (rand < edge.dp) {
                    if (debug) {
                        printAgent(i, "delay");
//                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") delay" << std::endl;
                    }
//                    nodeAgentMap[nextNodeId] = i;
                } else {
                    if (outputFile.is_open()) {
                        outputFile << i << " " << graph.getNode(nextNodeId).index << std::endl;
                    }
                    if (debug) {
                        printAgent(i, "");
//                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ")->("
//                                  << state + 1 << "," << nextNodeId << ")" << std::endl;
                    }
                    unblocked.erase(i);
                    moved.insert(i);
                    nodeAgentMap.erase(currentNodeId);
//                    nodeAgentMap[nextNodeId] = i;
                    agents[i].current = nextNodeId;
                    agents[i].waitingTimestep = 0;
                    state++;
                    updateSharedNode(currentNodeId, i, state);
                    if (state + 1 >= pathTopoNodeIds[i].size()) {
                        if (firstAgentArrivingTimestep == 0) {
                            firstAgentArrivingTimestep = currentTimestep;
                        }
                        ++count;
                    }
                }*/
            }
            /*else {
                if (debug) {
                    std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") error" << std::endl;
                }
                assert(0);
            }*/
        }

        if (count >= agents.size()) {
            break;
        }

        advanceTimestep(currentTimestep);

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

bool ContinuousOnlineSimulator::isEdgeInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
    auto neighborEdges = boost::out_edges(nodeId1, topoGraph);
    return std::any_of(neighborEdges.first, neighborEdges.second, [&](const auto &edge){
        return edge.m_target == nodeId2;
    });
//    for (auto edge: boost::make_iterator_range(neighborEdges)) {
//        unsigned int neighborNodeId = edge.m_target;
//        if (neighborNodeId == nodeId2) {
//            return true;
//        }
//    }
//    return false;
}

void ContinuousOnlineSimulator::initSharedNodes(size_t i, size_t j) {
    std::unordered_map<unsigned int, std::vector<std::pair<size_t, unsigned int> > > m;
    for (auto x: {i, j}) {
        for (size_t k = 0; k < paths[x].size(); k++) {
            // node states: k * 2
            m[paths[x][k]].emplace_back(x, k * 2);
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


Graph::NodeEdgeState ContinuousOnlineSimulator::getNodeEdgeState(size_t agentId, unsigned int state) {
    if (state % 2 == 0) {
        return Graph::NodeEdgeState{true, sqrt(2) / 4, paths[agentId][state / 2], 0};
    } else {
        return Graph::NodeEdgeState{false, sqrt(2) / 4, paths[agentId][state / 2], paths[agentId][state / 2 + 1]};
    }
}

void ContinuousOnlineSimulator::initSharedStates(size_t agentId1, size_t agentId2) {
    for (unsigned int i = 0; i < paths[agentId1].size() * 2 - 1; i++) {
        auto si = getNodeEdgeState(agentId1, i);
        for (unsigned int j = 0; j < paths[agentId2].size() * 2 - 1; j++) {
            auto sj = getNodeEdgeState(agentId2, j);
            if (graph.isConflict(si, sj)) {
                SharedNodePair pair{agentId1, i, agentId2, j};
                sharedStates.emplace_back(pair);
            }
        }
    }
}


std::vector<ContinuousOnlineSimulator::SDGEdge>
ContinuousOnlineSimulator::updateSharedNode(size_t agentId, unsigned int state, bool dryRun) {
/*    auto it = sharedNodes.find(nodeId);
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
    }*/
//    sdgData[agentId][state].conflicts.clear();
//    sdgData[agentId][state].determinedEdgeSources.clear();
//    sdgData[agentId][state].unsettledEdgePairs.clear();
//    if (debug) {
//        std::cout << "update unsettled edge pairs for (" << agentId << "," << state << ")" << std::endl;
//        std::cout << "unsettled size before update: " << unsettledEdgePairsSet.size() << std::endl;
//    }
    SDGNode node = {agentId, state};
    std::vector<SDGEdge> result;
/*    if (debug) {
        std::cout << "update unsettled edge pairs for " << node << std::endl;
    }*/
    for (auto &pair: sdgData[agentId][state].unsettledEdgePairs) {
        auto it = unsettledEdgePairsSet.find(pair);
        if (it != unsettledEdgePairsSet.end()) {
            // first time to remove the pair
            SDGEdge selectedEdge = {};
            if (pair.first.dest == node) {
                selectedEdge = pair.second;
            } else if (pair.second.dest == node) {
                selectedEdge = pair.first;
            } else {
                std::cerr << "unsettled edge pair error" << std::endl;
                exit(-1);
            }
/*            if (debug) {
                std::cout << "try to remove unsettled edge pair: " << pair.first << " " << pair.second << std::endl;
                std::cout << "try to add settled edge: " << selectedEdge << std::endl;
            }*/
/*            if (agents[selectedEdge.source.agentId].state < selectedEdge.source.state) {
                continue;
            }*/
            if (debug) {
                std::cout << "select unsettled edge pair: " << pair.first << " " << pair.second << " -> "
                          << selectedEdge << std::endl;
            }
            auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(selectedEdge);
            if (isEdgeInTopoGraph(nodeId1, nodeId2)) {
                continue;
            }
            result.emplace_back(selectedEdge);
            if (isPathInTopoGraph(nodeId2, nodeId1)) {
                std::cerr << "error: path exists in topo graph!" << std::endl;
//                exit(-1);
            }
            if (!dryRun) {
                boost::add_edge(nodeId1, nodeId2, topoGraph);
                unsettledEdgePairsSet.erase(it);
                sdgData[selectedEdge.dest.agentId][selectedEdge.dest.state].determinedEdgeSources.push_back(
                        {selectedEdge.source.agentId, selectedEdge.source.state});
            }
        }
    }
//    if (debug) {
//        std::cout << "unsettled size after update: " << unsettledEdgePairsSet.size() << std::endl;
//    }
    return result;
}


void ContinuousOnlineSimulator::initSimulation() {
    blocked.clear();
    paths.clear();
    paths.resize(agents.size());
    deadEndStates.clear();
    deadEndStates.resize(agents.size());
    pathTopoNodeIds.clear();
    pathTopoNodeIds.resize(agents.size());
    nodeAgentMap.clear();
    topoGraph.clear();
    sdgData.resize(agents.size());

    initDelayedIntervals();

    unsigned int topoGraphNodeNum = 0;
    for (size_t i = 0; i < agents.size(); i++) {
        agents[i].current = agents[i].start;
        agents[i].state = 0;
        agents[i].timestep = 0;
        blocked.insert(i);
        nodeAgentMap[agents[i].current] = i;
        for (size_t j = 0; j < solver->solution->plans[i]->path.size(); j++) {
            auto newNodeId = solver->solution->plans[i]->path[j].nodeId;
            if (paths[i].empty() || paths[i].back() != newNodeId) {
                paths[i].emplace_back(newNodeId);
                if (!pathTopoNodeIds[i].empty()) {
                    // add an edge state to topo graph
                    pathTopoNodeIds[i].emplace_back(boost::add_vertex(topoGraph));
                }
                // add a node state to topo graph
                pathTopoNodeIds[i].emplace_back(boost::add_vertex(topoGraph));
            }
        }
        // when j % 2 == 0, it is a node state; when j % 2 == 1, it is an edge state
        for (size_t j = 0; j < pathTopoNodeIds[i].size() - 1; j++) {
            boost::add_edge(pathTopoNodeIds[i][j], pathTopoNodeIds[i][j + 1], topoGraph);
        }
        topoGraphNodeNum += pathTopoNodeIds[i].size();
        sdgData[i].resize(pathTopoNodeIds[i].size());
    }


    for (size_t i = 0; i < agents.size(); i++) {
        for (size_t j = i + 1; j < agents.size(); j++) {
//            initSharedNodes(i, j);
            initSharedStates(i, j);
        }
    }

/*    for (const auto &[nodeId, sharedNode]: sharedNodes) {
        for (const auto &sharedNodePair: sharedNode) {
            // TODO: really need to duplicate?
            sdgData[sharedNodePair.agentId1][sharedNodePair.state1].conflicts.push_back(
                    {sharedNodePair.agentId2, sharedNodePair.state2}
            );
            sdgData[sharedNodePair.agentId2][sharedNodePair.state2].conflicts.push_back(
                    {sharedNodePair.agentId1, sharedNodePair.state1}
            );
        }
    }*/
    for (const auto &sharedNodePair: sharedStates) {
        // TODO: really need to duplicate?
        sdgData[sharedNodePair.agentId1][sharedNodePair.state1].conflicts.push_back(
                {sharedNodePair.agentId2, sharedNodePair.state2}
        );
        sdgData[sharedNodePair.agentId2][sharedNodePair.state2].conflicts.push_back(
                {sharedNodePair.agentId1, sharedNodePair.state1}
        );
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
    generateUnsettledEdges();

    for (size_t i = 0; i < sdgData.size(); i++) {
        for (unsigned int state = 0; state < sdgData[i].size(); state++) {
            const auto &item = sdgData[i][state];
            for (const auto &edgePair: item.unsettledEdgePairs) {
                unsettledEdgePairsSet.emplace(edgePair);
            }
            for (const auto &node: item.determinedEdgeSources) {
                SDGEdge selectedEdge = {node, {i, state}};
                if (debug) {
                    std::cout << "add settled edge: " << selectedEdge << std::endl;
                }
                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(selectedEdge);
                if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                }
            }
        }
    }

/*    for (size_t i = 0; i < agents.size(); i++) {
        if (!paths[i].empty()) {
            for (size_t j = 0; j < agents.size(); j++) {
                if (i != j) {
                    SDGEdge edge = {{i, 1}, {j, 0}};
                    if (debug) {
                        std::cout << "add settled edge: " << edge << std::endl;
                    }
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                }
            }
        }
    }*/
}


void ContinuousOnlineSimulator::initChecks() {
    ready.clear();
/*    std::set<unsigned int> dests;
    auto edges = boost::edges(topoGraph);
    for (auto it = edges.first; it != edges.second; ++it) {
        auto dest = boost::target(*it, topoGraph);
        dests.emplace(dest);
    }
    for (const auto &[nodeId, sharedNode]: sharedNodes) {
        for (auto it = sharedNode.begin(); it != sharedNode.end(); ++it) {
            if (it->state2 > agents[it->agentId2].state) {
//                if (it->state1 + 1 < paths[it->agentId1].size() && it->state2 > agents[it->agentId2].state) {
                auto dest = pathTopoNodeIds[it->agentId2][it->state2];
                dests.emplace(dest);
//                }
            }
            if (it->state1 > agents[it->agentId1].state) {
//                if (it->state2 + 1 < paths[it->agentId2].size() && it->state1 > agents[it->agentId1].state) {
                auto dest = pathTopoNodeIds[it->agentId1][it->state1];
                dests.emplace(dest);
//                }
            }
        }
    }

    keepMoving.clear();
    for (auto it = moved.begin(); it != moved.end();) {
        auto &agent = agents[*it];
        if (agent.current != agent.goal && agent.state + 1 < pathTopoNodeIds[*it].size()) {
            auto dest = pathTopoNodeIds[*it][agent.state + 1];
            if (dests.find(dest) == dests.end()) {
                keepMoving.emplace(*it);
//                unblocked.emplace(*it);
//                it = moved.erase(it);
//                continue;
            }
        }
        ++it;
    }*/

    for (auto i: boost::join(blocked, moved)) {
        auto &agent = agents[i];
        if (agent.current != agent.goal && agent.state + 1 < pathTopoNodeIds[i].size()) {
            ready.insert(i);
        }
    }
    blocked.clear();
//    moved.clear();
    unshared.clear();
//    for (auto i: delayed_set) {
//        if (ready.find(i) != ready.end()) {
//            ready.erase(i);
//            blocked.insert(i);
//        }
//    }
}


void ContinuousOnlineSimulator::unnecessaryBlockCheck() {

}


//void ContinuousOnlineSimulator::unsharedCheck() {
//    for (auto it = ready.begin(); it != ready.end();) {
//        auto &agent = agents[*it];
//        auto nextNodeId = paths[*it][agent.state / 2 + 1];
//        auto it2 = sharedNodes.find(nextNodeId);
//        if (it2 == sharedNodes.end() || it2->second.empty()) {
//            unshared.insert(*it);
//            it = ready.erase(it);
//        } else {
//            ++it;
//        }
//    }
//}
//
//
//void ContinuousOnlineSimulator::neighborCheck() {
//    for (auto it = ready.begin(); it != ready.end();) {
//        auto &agent = agents[*it];
//        auto nextNodeId = paths[*it][agent.state / 2 + 1];
//        auto it2 = nodeAgentMap.find(nextNodeId);
//        if (it2 != nodeAgentMap.end() && *it != it2->second) {
//            blocked.insert(*it);
//            it = ready.erase(it);
//        } else {
//            ++it;
//        }
//    }
//}
//
//void ContinuousOnlineSimulator::deadEndCheck() {
//    for (auto it = ready.begin(); it != ready.end();) {
//        bool remove = false;
//        if (agents[*it].state / 2 == paths[*it].size() - 2) {
//            for (auto &&[j, state]: deadEndStates[*it]) {
//                if (agents[j].state <= state) {
//                    remove = true;
//                    break;
//                }
//            }
//        }
//        if (remove) {
//            blocked.insert(*it);
//            it = ready.erase(it);
//        } else {
//            ++it;
//        }
//    }
//}

void ContinuousOnlineSimulator::singleAgentCheck() {
    for (auto it = ready.begin(); it != ready.end();) {
        auto i = *it;
        bool fail = false;
        if (agents[i].state + 1 >= pathTopoNodeIds[i].size() || agents[i].state % 2 == 1) {
            // should not happen?
            fail = true;
        } else {
//            auto addedEdges = updateSharedNode(i, agents[i].state + 1, true);
//            for (const auto &edge: addedEdges) {
//                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
//                boost::add_edge(nodeId1, nodeId2, topoGraph);
//            }
            for (size_t j = 0; j < agents.size(); j++) {
                if (i == j || agents[j].state + 1 >= pathTopoNodeIds[j].size()) {
                    continue;
                }
                SDGEdge edge = {{i, agents[i].state + 2},
                                {j, agents[j].state + 1}};
//                std::cout << "test:" << edge << std::endl;
                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                if (isPathInTopoGraph(nodeId2, nodeId1)) {
                    fail = true;
                    break;
                }
                // we can also do a feasibility check here!
            }
//            for (const auto &edge: addedEdges) {
//                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
//                boost::remove_edge(nodeId1, nodeId2, topoGraph);
//            }
        }
        if (fail) {
            it = ready.erase(it);
            blocked.insert(i);
        } else {
            ++it;
        }
    }
}

void ContinuousOnlineSimulator::naiveCycleCheckHelper(std::vector<size_t> &readyList, size_t length, size_t start,
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
        std::vector<unsigned int> oldStates(agents.size());
        for (auto i: checkedReady) {
            oldStates[i] = agents[i].state;
            agents[i].state = (agents[i].state / 2 + 1) * 2;
        }
        // perform a neighbor check here
        auto agentId1 = agents.size(), agentId2 = agents.size();
        for (size_t i = 0; i < agents.size(); i++) {
            auto nodeId = paths[i][agents[i].state / 2];
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
            agents[i].state = oldStates[i];
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

void ContinuousOnlineSimulator::naiveCycleCheck() {
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
void ContinuousOnlineSimulator::heuristicCycleCheck() {
//    std::cout << "start cycle check: ";
    std::set<size_t> newReady = ready;
    std::vector<std::vector<SDGEdge>> addedEdges(agents.size());
    std::map<SDGEdge, unsigned int> addedEdgesCounter;
    cycleCheckAddedEdges.clear();

    auto updateAgentTopoGraph = [&](size_t agentId) {
//        addedEdges[agentId] = updateSharedNode(agentId, agents[agentId].state, true);
//        for (const auto &edge: addedEdges[agentId]) {
//            auto it = addedEdgesCounter.find(edge);
//            if (it == addedEdgesCounter.end()) {
//                it = addedEdgesCounter.emplace_hint(it, edge, 0);
//                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
//                boost::add_edge(nodeId1, nodeId2, topoGraph);
//                cycleCheckAddedEdges.emplace(edge);
//                if (debug) {
//                    std::cout << "temporarily add settled edge: " << edge << std::endl;
//                }
//            }
//            it->second++;
//        }
    };

    auto recoverAgentTopoGraph = [&](size_t agentId) {
//        for (const auto &edge: addedEdges[agentId]) {
//            auto it = addedEdgesCounter.find(edge);
//            it->second--;
//            if (it->second == 0) {
//                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
//                boost::remove_edge(nodeId1, nodeId2, topoGraph);
//                cycleCheckAddedEdges.erase(edge);
//                if (debug) {
//                    std::cout << "temporarily remove settled edge: " << edge << std::endl;
//                }
//            }
//        }
    };

    // save old states for recovery
    std::vector<unsigned int> oldStates(agents.size());
    for (auto i: ready) {
        if (agents[i].state % 2 != 0) {
            std::cerr << "ready agents must be in node state" << std::endl;
            exit(-1);
        }
        oldStates[i] = agents[i].state;
        agents[i].state++;
        updateAgentTopoGraph(i);
//        agents[i].state = getNextNodeState(agents[i].state);
//        std::cout << "(" << i << "," << agents[i].state << ") ";
    }
//    std::cout << std::endl;

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
            // recover state of agent 1 and remove it from ready
            newReady.erase(agentId1);
            agents[agentId1].state = oldStates[agentId1];
            removed.insert(agentId1);
            recoverAgentTopoGraph(agentId1);
        } else if (newReady.find(agentId2) != newReady.end()) {
            // recover state of agent 2 and remove it from ready
            newReady.erase(agentId2);
            agents[agentId2].state = oldStates[agentId2];
            removed.insert(agentId2);
            recoverAgentTopoGraph(agentId2);
        } else {
            // recover states of all remaining agents in ready and clear ready
            for (auto i: newReady) {
                agents[i].state = oldStates[i];
                removed.insert(i);
                recoverAgentTopoGraph(i);
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
            updateAgentTopoGraph(i);
//            agents[i].state = getNextNodeState(agents[i].state);
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
                agents[i].state = oldStates[i];
                recoverAgentTopoGraph(i);
            }
        }
    }
//    }

    // hope this not happen
    if (newReady.empty() && unblocked.empty() && unshared.empty()) {
        std::cerr << "error: ready and unblocked both empty!" << std::endl;
    }

    // recover topo graph
    for (auto i: newReady) {
        recoverAgentTopoGraph(i);
    }

    // recover all states for safety
    for (auto i: ready) {
        agents[i].state = oldStates[i];
        if (newReady.find(i) == newReady.end()) {
            blocked.insert(i);
        }
    }
    ready.swap(newReady);
}


void ContinuousOnlineSimulator::cycleCheck() {
    if (ready.empty()) return;
    if (firstAgentArrivingTimestep == 0) {
        cycleCheckCount++;
        cycleCheckAgents += ready.size();
    }
    // suppose all unblocked agents are at their next state (an edge state)
//    for (auto i: unblocked) {
//        agents[i].state++;
//    }
//    for (auto i: unshared) {
//        agents[i].state++;
//    }

    if (isHeuristicCycleCheck) {
        heuristicCycleCheck();
    } else {
        naiveCycleCheck();
    }

    // recover the states of the agents
//    for (auto i: unblocked) {
//        agents[i].state--;
//    }
//    for (auto i: unshared) {
//        agents[i].state--;
//    }
//    std::cerr << "end cycle check: ";
}

bool ContinuousOnlineSimulator::isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
    auto visitor = TopoGraphBFSVisitor(nodeId2);
    try {
        boost::breadth_first_search(topoGraph, nodeId1, boost::visitor(visitor));
    } catch (...) {
        return true;
    }
    return false;
}

/** Algorithm 1 **/
std::pair<size_t, size_t> ContinuousOnlineSimulator::feasibilityCheckHelper(
        std::list<SDGEdgePair> &unsettledEdgePairs,
        bool recursive
) {
    std::vector<SDGEdge> addedEdges;
//    std::cout << "feasibility check" << std::endl;

    /** line 1 **/
    while (!unsettledEdgePairs.empty()) {
        /** line 2 **/
        if (firstAgentArrivingTimestep == 0) {
            feasibilityCheckLoopCount++;
        }
        unsigned int erasedEdges = 0;
        /** line 3 **/
        for (auto it = unsettledEdgePairs.begin(); it != unsettledEdgePairs.end();) {
//            feasibilityCheckIterationTemp++;
            if (firstAgentArrivingTimestep == 0) {
                feasibilityCheckTopoCount++;
            }
            bool edge1Settled = it->second.dest.state <= agents[it->second.dest.agentId].state;
//                    || it->first.source.state + 1 >= pathTopoNodeIds[it->first.source.agentId].size();
            bool edge2Settled = it->first.dest.state <= agents[it->first.dest.agentId].state;
//                    || it->second.source.state + 1 >= pathTopoNodeIds[it->second.source.agentId].size();

            std::vector<SDGEdge> selectedEdges;
            unsigned int maxSelectedEdges = 0;


            if (!(edge1Settled && edge2Settled)) {
                if (!edge2Settled) {
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(it->first);
                    if (!isPathInTopoGraph(nodeId2, nodeId1)) {
                        selectedEdges.emplace_back(it->first);
                    }
                    ++maxSelectedEdges;
                }
                if (!edge1Settled) {
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(it->second);
                    if (!isPathInTopoGraph(nodeId2, nodeId1)) {
                        selectedEdges.emplace_back(it->second);
                    }
                    ++maxSelectedEdges;
                }
            } else {
                // not feasible, do nothing here
            }

            if (selectedEdges.empty()) {
                // no edge can be selected
                if (debug) {
                    std::cout << "no edge can be selected" << std::endl;
                    std::cout << it->first << " " << it->second << " "
                              << edge1Settled << " " << edge2Settled << std::endl;
                }
                for (auto &edge: addedEdges) {
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                    boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    if (debug) {
//                        std::cout << "temporarily remove settled edge: " << edge << std::endl;
                    }
                }
                return std::make_pair(it->first.dest.agentId, it->second.dest.agentId);
            } else if (selectedEdges.size() == 1) {
                auto &edge = selectedEdges[0];
                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                    addedEdges.emplace_back(edge);
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                    if (debug) {
//                        std::cout << "temporarily select unsettled edge pair: " << it->first << " " << it->second << " -> " << edge << std::endl;
                    }
                }
                if (debug) {
//                    std::cout << it->first << " " << it->second << " "
//                              << edge1Settled << " " << edge2Settled << std::endl;
                }
                it = unsettledEdgePairs.erase(it);
                ++erasedEdges;
//                std::cout << "choose: " << selectedEdges[0].first << " " << selectedEdges[0].second << std::endl;
            } else {
                ++it;
            }
/*
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
*/
        }

//        std::cout << erasedEdges << std::endl;

        /** line 10 **/
        if (erasedEdges == 0 && !unsettledEdgePairs.empty()) {
            /** line 11 **/
            auto it = unsettledEdgePairs.begin();
            auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(it->first);

            /** line 12 **/
            // TODO: add randomness here
            bool randomEdgeSelected = !isEdgeInTopoGraph(nodeId1, nodeId2);
            if (randomEdgeSelected) {
                addedEdges.emplace_back(it->first);
                boost::add_edge(nodeId1, nodeId2, topoGraph);
                if (debug) {
//                    std::cout << "temporarily traverse unsettled edge pair: " << it->first << " " << it->second << " -> " << it->first << std::endl;
                }
            }


            /** line 13 **/
            unsettledEdgePairs.erase(it);

            /** exhaustive version of Algorithm 1 **/
            if (recursive) {
                auto sharedNodesListBackup = unsettledEdgePairs;
//                std::cout << "recursive" << std::endl;
                auto result = feasibilityCheckHelper(sharedNodesListBackup, recursive);
                if (firstAgentArrivingTimestep == 0) {
                    feasibilityCheckRecursionCount++;
                }
                if (randomEdgeSelected) {
                    boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    if (debug) {
//                        std::cout << "temporarily remove settled edge: " << it->first << std::endl;
                    }
                    addedEdges.pop_back();
                }

                if (result.first == agents.size() && result.second == agents.size()) {
                    for (auto &edge: addedEdges) {
                        auto [_nodeId1, _nodeId2] = getTopoEdgeBySDGEdge(edge);
                        boost::remove_edge(_nodeId1, _nodeId2, topoGraph);
                        if (debug) {
//                            std::cout << "temporarily remove settled edge: " << edge << std::endl;
                        }
                    }
                    return result;
                }

                auto [nodeId3, nodeId4] = getTopoEdgeBySDGEdge(it->second);
                if (!isEdgeInTopoGraph(nodeId3, nodeId4)) {
                    addedEdges.emplace_back(it->second);
                    boost::add_edge(nodeId3, nodeId4, topoGraph);
                    if (debug) {
//                        std::cout << "temporarily traverse unsettled edge pair: " << it->first << " " << it->second << " -> " << it->second << std::endl;
                    }
                }
            }
//            std::cout << "random choose: " << nodeId1 << " " << nodeId2 << std::endl;
        }
    }
    for (auto &edge: addedEdges) {
        auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
        boost::remove_edge(nodeId1, nodeId2, topoGraph);
        if (debug) {
//            std::cout << "temporarily remove settled edge: " << edge << std::endl;
        }
    }
//    std::cout << "feasible" << std::endl;
    /** line 14 **/
    return std::make_pair(agents.size(), agents.size());
}


std::pair<size_t, size_t> ContinuousOnlineSimulator::feasibilityCheckTest(bool recursive) {
//    std::cerr << "start feasibility check" << std::endl;

//    std::list<SharedNodePair> sharedNodesList;
    std::list<SDGEdgePair> unsettledEdgePairs(unsettledEdgePairsSet.begin(), unsettledEdgePairsSet.end());

    // neighbor check, can be removed later
/*    std::unordered_map<size_t, size_t> nodeAgentMapSnapshot;
    for (size_t i = 0; i < agents.size(); i++) {
        std::vector<unsigned int> occupiedNodes = {paths[i][agents[i].state / 2]};
        if (agents[i].state % 2 == 1) {
            occupiedNodes.emplace_back(paths[i][agents[i].state / 2 + 1]);
        }
        for (auto nodeId : occupiedNodes) {
//            std::cout << nodeId << " " << i << " " << agents[i].state << std::endl;
            auto it = nodeAgentMapSnapshot.find(nodeId);
            if (it == nodeAgentMapSnapshot.end()) {
                nodeAgentMapSnapshot.emplace_hint(it, nodeId, i);
            } else {
//            std::cout << "neighbor check" << std::endl;
                return std::make_pair(it->second, i);
            }
        }
    }*/

    if (debug) {
        std::cout << "begin feasibility check, unsettled size: " << unsettledEdgePairs.size() << std::endl;
    }

    if (firstAgentArrivingTimestep == 0) {
        feasibilityCheckUnsettledCount += unsettledEdgePairs.size();
    }

//    return feasibilityCheckHelper(sharedNodesList, recursive);
//    for (const auto &[nodeId, sharedNode]: sharedNodes) {
//        for (const auto &sharedNodePair: sharedNode) {
//            sharedNodesList.emplace_back(sharedNodePair);
//        }
//    }

    return feasibilityCheckHelper(unsettledEdgePairs, recursive);

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

std::pair<size_t, size_t> ContinuousOnlineSimulator::feasibilityCheck() {
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

bool ContinuousOnlineSimulator::generateUnsettledEdges() {
    for (size_t agentId1 = 0; agentId1 < sdgData.size(); agentId1++) {
        for (unsigned int state1 = 0; state1 < sdgData[agentId1].size(); state1++) {
            auto &sdgDataItem = sdgData[agentId1][state1];
            for (const auto &conflict: sdgDataItem.conflicts) {
                size_t agentId2 = conflict.agentId;
                unsigned int state2 = conflict.state;
                // prevent duplication
                if (agentId1 < agentId2 || (agentId1 == agentId2 && state1 < state2)) {
                    SDGEdge edge1{}, edge2{};
//                    std::cout << agentId1 << " " << state1 << " " << agentId2 << " " << state2 << " ";
                    if (state1 % 2 == 0) {
                        edge1 = {SDGNode{agentId2, state2 + 2}, SDGNode{agentId1, state1 - 1}};
                    } else {
                        edge1 = {SDGNode{agentId2, state2 + 1}, SDGNode{agentId1, state1}};
                    }
                    if (state2 % 2 == 0) {
                        edge2 = {SDGNode{agentId1, state1 + 2}, SDGNode{agentId2, state2 - 1}};
                    } else {
                        edge2 = {SDGNode{agentId1, state1 + 1}, SDGNode{agentId2, state2}};
                    }
                    int invalid = 0;
                    if (edge1.source.state >= sdgData[edge1.source.agentId].size() ||
                        edge1.dest.state >= sdgData[edge1.dest.agentId].size()) {
                        // edge1 is not valid
                        invalid += 1;
                    }
                    if (edge2.source.state >= sdgData[edge2.source.agentId].size() ||
                        edge2.dest.state >= sdgData[edge2.dest.agentId].size()) {
                        // edge2 is not valid
                        invalid += 2;
                    }
                    if (invalid == 0) {
                        // record the pair of unsettled edges
                        SDGEdgePair edgePair;
                        if (edge1 < edge2) {
                            edgePair = {edge1, edge2};
                        } else {
                            edgePair = {edge2, edge1};
                        }
                        sdgData[edge1.dest.agentId][edge1.dest.state].unsettledEdgePairs.push_back(edgePair);
                        sdgData[edge2.dest.agentId][edge2.dest.state].unsettledEdgePairs.push_back(edgePair);
                        if (debug) {
//                            std::cout << "generate unsettled edges " << edge1 << " " << edge2 << std::endl;
                        }
                    } else if (invalid == 1) {
                        // edge2 is determined edge
                        sdgData[edge2.dest.agentId][edge2.dest.state].determinedEdgeSources.push_back(
                                {edge2.source.agentId, edge2.source.state});
                        if (debug) {
//                            std::cout << "generate determined edge " << edge2 << std::endl;
                        }
                    } else if (invalid == 2) {
                        // edge1 is determined edge
                        sdgData[edge1.dest.agentId][edge1.dest.state].determinedEdgeSources.push_back(
                                {edge1.source.agentId, edge1.source.state});
                        if (debug) {
//                            std::cout << "generate determined edge " << edge1 << std::endl;
                        }
                    } else {
                        // not solvable
                        return false;
                    }
                }
            }
        }
    }
    return true;
}



