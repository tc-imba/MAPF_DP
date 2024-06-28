//
// Created by liu on 7/1/2022.
//

#include <boost/graph/topological_sort.hpp>
#include <boost/range/join.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

#include "ContinuousOnlineSimulator.h"


unsigned int ContinuousOnlineSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                                                 unsigned int delayStart, unsigned int delayInterval) {
    if (debug) {
        std::ostringstream oss;
        for (unsigned int i = 0; i < agents.size(); i++) {
            //        if (agents[i].current == agents[i].goal) continue;
            oss.str("");
            oss.clear();
            oss << "agent " << i << " (" << agents[i].start << "->" << agents[i].goal << "): ";
            for (const auto &label: solver->solution->plans[i]->path) {
                auto &node = graph.getNode(label.nodeId);
                oss << "(" << label.state * 2 << "," << label.nodeId << "," << node.x << "," << node.y << ")->";
            }
            SPDLOG_DEBUG("{}", oss.str());
            //        nodeStates[agents[i].start] = 0;
        }
    }

    initSimulation();
    executionTimeVec.clear();

    // check whether the initial plans is feasible
    auto start = std::chrono::steady_clock::now();
    auto [agentId1, agentId2] = feasibilityCheck();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cerr << elapsed_seconds.count() << std::endl;
    if (agentId1 != agents.size() || agentId2 != agents.size()) {
        std::cerr << "initial plans is not feasible" << std::endl;
        exit(-1);
    }
    if (snapshot) {
        depGraph.addSavedEdges();
    }


    //    std::uniform_real_distribution<double> distribution(0, 1);


    while (currentTimestep < maxTimeStep) {
        SPDLOG_DEBUG("begin timestep {}", currentTimestep);

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
        double newCurrentTimestep = currentTimestep + deltaTimestep;
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &state = agents[i].state;
            if (state + 1 < depGraph.pathTopoNodeIds[i].size() && state % 2 == 1 &&
                agents[i].arrivingTimestep <= newCurrentTimestep) {

//                newCurrentTimestep = std::max(newCurrentTimestep, agents[i].arrivingTimestep);
                auto currentNodeId = paths[i][state / 2];
                auto nextNodeId = paths[i][state / 2 + 1];

                //                if (outputFile.is_open()) {
                //                    outputFile << i << " " << graph.getNode(nextNodeId).index << std::endl;
                //                }

                if (debug) {
                    printAgent(i, "");
                }
                unblocked.erase(i);
                moved.insert(i);
                nodeAgentMap.erase(currentNodeId);
                agents[i].current = nextNodeId;
                agents[i].timestep = agents[i].arrivingTimestep;
                ++state;
                depGraph.updateSharedNode(i, state);
                if (state + 1 >= depGraph.pathTopoNodeIds[i].size()) {
                    if (firstAgentArrivingTimestep == 0) {
                        firstAgentArrivingTimestep = agents[i].arrivingTimestep;
                    }
                }
            }
        }
        currentTimestep = newCurrentTimestep;

        SPDLOG_DEBUG("expanded timestep {}", currentTimestep);

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
        printSets("init        |", currentTimestep);
        singleAgentCheck();

        if (!snapshot) {
            printSets("cycle       |", currentTimestep);
            cycleCheck();
        }
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
        printSets("final       |", currentTimestep);

        //        std::cout << unblocked.size() << " " << savedReady.size() + savedUnshared.size() << std::endl;

        //        for (auto i: savedBlocked) {
        //            if (ready.find(i) != ready.end()) {
        //                exit(0);
        //            }
        //        }

        std::chrono::duration<double> elapsed_seconds = end - start;
        if (firstAgentArrivingTimestep == 0) {
            unblockedAgents += unblocked.size();
            firstAgentArrivingExecutionTime += elapsed_seconds.count();
        }
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
            auto currentNodeId = paths[i][state / 2];
            //            if (debug) {
            //                std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ") initial" << std::endl;
            //            }

            if (state + 1 >= depGraph.pathTopoNodeIds[i].size()) {
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
                    if (!outputPaths[i].empty() && outputPaths[i].back().agentState != AgentState::BLOCK) {
                        outputPaths[i].push_back(PathNode{currentTimestep, state, AgentState::BLOCK});
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


                    outputPaths[i].push_back(PathNode{currentTimestep, state, AgentState::UNBLOCK});
                    ++state;
                    agents[i].arrivingTimestep = getAgentArrivingTime(currentTimestep, delayInterval, i, edge);
                    if (debug) {
                        printAgent(i, "");
                        //                        std::cout << "agent " << i << ": (" << state << "," << currentNodeId << ")->("
                        //                                  << state + 1 << "," << currentNodeId << "-" << nextNodeId << ")" << std::endl;
                    }
                    nodeAgentMap[nextNodeId] = i;
                    depGraph.updateSharedNode(i, state);
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
                    if (state + 1 >= depGraph.pathTopoNodeIds[i].size()) {
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

    for (unsigned int i = 0; i < agents.size(); i++) {
        if (agents[i].state + 1 >= depGraph.pathTopoNodeIds[i].size()) {
            outputPaths[i].push_back(PathNode{agents[i].timestep, agents[i].state, AgentState::COMPLETE});
        } else {
            outputPaths[i].push_back(PathNode{currentTimestep, agents[i].state, AgentState::FAIL});
        }
    }

    printPaths(currentTimestep);

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


void ContinuousOnlineSimulator::initSimulation() {
    blocked.clear();
    //    paths.clear();
    //    paths.resize(agents.size());
    //    deadEndStates.clear();
    //    deadEndStates.resize(agents.size());
    //    depGraph.pathTopoNodeIds.clear();
    //    depGraph.pathTopoNodeIds.resize(agents.size());
    nodeAgentMap.clear();
    //    topoGraph.clear();
    //    sdgData.resize(agents.size());
    outputPaths.clear();
    outputPaths.resize(agents.size());

    initDelayedIntervals();

    unsigned int topoGraphNodeNum = 0;
    for (size_t i = 0; i < agents.size(); i++) {
        agents[i].current = agents[i].start;
        agents[i].state = 0;
        agents[i].timestep = 0;
        blocked.insert(i);
        nodeAgentMap[agents[i].current] = i;
        /*for (size_t j = 0; j < solver->solution->plans[i]->path.size(); j++) {
            auto newNodeId = solver->solution->plans[i]->path[j].nodeId;
            if (paths[i].empty() || paths[i].back() != newNodeId) {
                paths[i].emplace_back(newNodeId);
                if (!depGraph.pathTopoNodeIds[i].empty()) {
                    // add an edge state to topo graph
                    depGraph.pathTopoNodeIds[i].emplace_back(boost::add_vertex(topoGraph));
                }
                // add a node state to topo graph
                depGraph.pathTopoNodeIds[i].emplace_back(boost::add_vertex(topoGraph));
            }
        }
        // when j % 2 == 0, it is a node state; when j % 2 == 1, it is an edge state
        for (size_t j = 0; j < depGraph.pathTopoNodeIds[i].size() - 1; j++) {
            boost::add_edge(depGraph.pathTopoNodeIds[i][j], depGraph.pathTopoNodeIds[i][j + 1], topoGraph);
        }
        topoGraphNodeNum += depGraph.pathTopoNodeIds[i].size();
        sdgData[i].resize(depGraph.pathTopoNodeIds[i].size());*/
    }


    /*    for (size_t i = 0; i < agents.size(); i++) {
        for (size_t j = i + 1; j < agents.size(); j++) {
//            initSharedNodes(i, j);
            initSharedStates(i, j);
        }
    }*/

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
    /*    for (const auto &sharedNodePair: sharedStates) {
        // TODO: really need to duplicate?
        sdgData[sharedNodePair.agentId1][sharedNodePair.state1].conflicts.push_back(
                {sharedNodePair.agentId2, sharedNodePair.state2}
        );
        sdgData[sharedNodePair.agentId2][sharedNodePair.state2].conflicts.push_back(
                {sharedNodePair.agentId1, sharedNodePair.state1}
        );
    }*/

    firstAgentArrivingExecutionTime = 0;
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
    //    generateUnsettledEdges();

    /*    for (size_t i = 0; i < sdgData.size(); i++) {
        for (unsigned int state = 0; state < sdgData[i].size(); state++) {
            const auto &item = sdgData[i][state];
            for (const auto &edgePair: item.unsettledEdgePairs) {
                unsettledEdgePairsSet.emplace(edgePair);
            }
            for (const auto &node: item.determinedEdgeSources) {
                SDGEdge selectedEdge = {node, {i, state}};
                SPDLOG_DEBUG("add settled edge: {}", selectedEdge);
                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(selectedEdge);
                if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                }
            }
        }
    }*/

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
    depGraph.solver = solver;
    depGraph.debug = debug;
    depGraph.snapshotOrder = snapshotOrder;
    depGraph.removeRedundant = removeRedundant;
    depGraph.useGroup = useGroup;
    depGraph.init();
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
                auto dest = depGraph.pathTopoNodeIds[it->agentId2][it->state2];
                dests.emplace(dest);
//                }
            }
            if (it->state1 > agents[it->agentId1].state) {
//                if (it->state2 + 1 < paths[it->agentId2].size() && it->state1 > agents[it->agentId1].state) {
                auto dest = depGraph.pathTopoNodeIds[it->agentId1][it->state1];
                dests.emplace(dest);
//                }
            }
        }
    }

    keepMoving.clear();
    for (auto it = moved.begin(); it != moved.end();) {
        auto &agent = agents[*it];
        if (agent.current != agent.goal && agent.state + 1 < depGraph.pathTopoNodeIds[*it].size()) {
            auto dest = depGraph.pathTopoNodeIds[*it][agent.state + 1];
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
        if (agent.current != agent.goal && agent.state + 1 < depGraph.pathTopoNodeIds[i].size()) {
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
        if (agents[i].state + 1 >= depGraph.pathTopoNodeIds[i].size() || agents[i].state % 2 == 1) {
            // should not happen?
            fail = true;
        } else {
            //            auto addedEdges = updateSharedNode(i, agents[i].state + 1, true);
            //            for (const auto &edge: addedEdges) {
            //                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
            //                boost::add_edge(nodeId1, nodeId2, topoGraph);
            //            }
            for (size_t j = 0; j < agents.size(); j++) {
                if (i == j || agents[j].state + 1 >= depGraph.pathTopoNodeIds[j].size()) {
                    continue;
                }
                NodeEdgeDependencyGraph::SDGEdge edge = {{i, agents[i].state + 2},
                                                         {j, agents[j].state + 1}};
                //                std::cout << "test:" << edge << std::endl;
                auto [nodeId1, nodeId2] = depGraph.getTopoEdgeBySDGEdge(edge);
                if (depGraph.isPathInTopoGraph(nodeId2, nodeId1)) {
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
    std::vector<std::vector<NodeEdgeDependencyGraph::SDGEdge>> addedEdges(agents.size());
    std::map<NodeEdgeDependencyGraph::SDGEdge, unsigned int> addedEdgesCounter;
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
        //                SPDLOG_DEBUG("cycle check add settled edge: {}", edge);
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
        //                SPDLOG_DEBUG("cycle check remove settled edge: {}", edge);
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
        SPDLOG_ERROR("error: ready and unblocked both empty!");
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


std::pair<size_t, size_t> ContinuousOnlineSimulator::feasibilityCheck() {
    if (firstAgentArrivingTimestep == 0) {
        feasibilityCheckCount++;
    }
    if (!isFeasibilityType) {
        return depGraph.feasibilityCheckTest(!isHeuristicFeasibilityCheck, true);
    }
    //    feasibilityCheckIterationTemp = 0;
    auto heuristicResult = depGraph.feasibilityCheckTest(false, true);
    //    feasibilityCheckIteration[0] += feasibilityCheckIterationTemp;
    //    feasibilityCheckIterationTemp = 0;
    auto exhaustiveResult = depGraph.feasibilityCheckTest(true, true);
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

void ContinuousOnlineSimulator::print(std::ostream &out) const {
    OnlineSimulator::print(out);
    out << "," << depGraph.numAllNodePairs << "," << depGraph.numFixedNodePairs << "," << depGraph.numAddedNodePairs;
}

void ContinuousOnlineSimulator::printPaths(double maxTimeStep) {
    if (!outputFileName.empty()) outputFile.open(outputFileName);
    if (!outputFile.is_open()) return;
    for (unsigned int i = 0; i < agents.size(); i++) {
        outputFile << i << " " << outputPaths[i].size() << std::endl;
        for (const auto &row: outputPaths[i]) {
            if (row.timestep <= maxTimeStep) {
                outputFile << row.state << " " << row.timestep << " " << paths[i][row.state / 2] << " " << AgentStateLiterals[(int) row.agentState] << std::endl;
            }
        }
    }
    outputFile.close();
}