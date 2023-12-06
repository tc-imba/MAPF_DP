//
// Created by liuyh on 4/12/2023.
//

#include "NodeDependencyGraph.h"

void NodeDependencyGraph::init() {
    paths.clear();
    paths.resize(agents.size());
    timestamps.clear();
    timestamps.resize(agents.size());
    deadEndStates.clear();
    deadEndStates.resize(agents.size());
    pathTopoNodeIds.clear();
    pathTopoNodeIds.resize(agents.size());
    topoGraph.clear();

    unsigned int topoGraphNodeNum = 0;
    for (size_t i = 0; i < agents.size(); i++) {
        for (size_t j = 0; j < solver->solution->plans[i]->path.size(); j++) {
            auto newNodeId = solver->solution->plans[i]->path[j].nodeId;
            auto newNodeTimestamp = solver->solution->plans[i]->path[j].estimatedTime;
            SPDLOG_DEBUG("{} {} {} {}", i, j, newNodeId, newNodeTimestamp);
            if (paths[i].empty() || paths[i].back() != newNodeId) {
                paths[i].emplace_back(newNodeId);
                timestamps[i].emplace_back(newNodeTimestamp);
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
}

NodeDependencyGraph::SDGEdgePair NodeDependencyGraph::makeSDGEdgePair(size_t agentId1, unsigned int state1, size_t agentId2, unsigned int state2) {
    SDGEdge edge1 = {SDGNode{agentId2, state2 + 1}, SDGNode{agentId1, state1}};
    SDGEdge edge2 = {SDGNode{agentId1, state1 + 1}, SDGNode{agentId2, state2}};
    if (edge1 < edge2) {
        return std::make_pair(edge1, edge2);
    } else {
        return std::make_pair(edge2, edge1);
    }
}

void NodeDependencyGraph::initSharedNodes(size_t i, size_t j) {
    std::unordered_map<unsigned int, std::vector<std::pair<size_t, unsigned int>>> m;
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

void NodeDependencyGraph::updateSharedNode(unsigned int nodeId, size_t agentId, unsigned int state) {
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

std::vector<NodeDependencyGraph::SDGEdge> NodeDependencyGraph::feasibilityCheckEdgePair(const SDGEdgePair &edgePair) {
    std::vector<SDGEdge> selectedEdges;
    unsigned int maxSelectedEdges = 0;

    // it->agentId1 = edgePair.first.source.agentId
    // it->agentId2 = edgePair.first.dest.agentId
    // it->state1 + 1 = edgePair.first.source.state
    // it->state2 = edgePair.first.dest.state

    //            if (it->state2 > agents[it->agentId2].state) {
    if (edgePair.first.dest.state > agents[edgePair.first.dest.agentId].state) {
        //                if (it->state1 + 1 == paths[it->agentId1].size()) {
        if (edgePair.first.source.state == paths[edgePair.first.source.agentId].size()) {
            ++maxSelectedEdges;
            //                } else if (it->state1 + 1 < paths[it->agentId1].size() && it->state2 > agents[it->agentId2].state) {
        } else if (edgePair.first.source.state < paths[edgePair.first.source.agentId].size() && edgePair.first.dest.state > agents[edgePair.first.dest.agentId].state) {
            //                    auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
            //                    auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];
            auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edgePair.first);
            if (!isPathInTopoGraph(nodeId2, nodeId1)) {
                selectedEdges.emplace_back(edgePair.first);
            }
            ++maxSelectedEdges;
        }
    }

    // it->agentId1 = edgePair.second.dest.agentId
    // it->agentId2 = edgePair.second.source.agentId
    // it->state1 = edgePair.second.dest.state
    // it->state2 + 1 = edgePair.second.source.state

    //            if (it->state1 > agents[it->agentId1].state) {
    if (edgePair.second.dest.state > agents[edgePair.second.dest.agentId].state) {
        //                if (it->state2 + 1 == paths[it->agentId2].size()) {
        if (edgePair.second.source.state == paths[edgePair.second.source.agentId].size()) {
            ++maxSelectedEdges;
            //                } else if (it->state2 + 1 < paths[it->agentId2].size() && it->state1 > agents[it->agentId1].state) {
        } else if (edgePair.second.source.state < paths[edgePair.second.source.agentId].size() && edgePair.second.dest.state > agents[edgePair.second.dest.agentId].state) {
            //                    auto nodeId1 = pathTopoNodeIds[it->agentId2][it->state2 + 1];
            //                    auto nodeId2 = pathTopoNodeIds[it->agentId1][it->state1];
            auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edgePair.second);
            if (!isPathInTopoGraph(nodeId2, nodeId1)) {
                selectedEdges.emplace_back(edgePair.second);
            }
            ++maxSelectedEdges;
        }
    }
    return selectedEdges;
}

/** Algorithm 1 **/
std::pair<size_t, size_t> NodeDependencyGraph::feasibilityCheckHelper(
        std::list<SDGEdgePair> &unsettledEdgePairs,
        bool recursive, bool speed) {
    std::vector<std::pair<SDGEdgePair, SDGEdge>> addedEdges;
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
            auto &edgePair = *it;
            //            feasibilityCheckIterationTemp++;

            if (firstAgentArrivingTimestep == 0) {
                feasibilityCheckTopoCount++;
            }
            auto selectedEdges = feasibilityCheckEdgePair(edgePair);

            if (selectedEdges.empty()) {
                /** line 4 **/
                //                if (maxSelectedEdges > 0) {
                // failed
                //                    std::cout << "failed: " << it->agentId1 << " " << it->state1 << " " << it->agentId2 << ""
                //                              << it->state2 << std::endl;
                for (auto [_edgePair, _edge]: addedEdges) {
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(_edge);
                    boost::remove_edge(nodeId1, nodeId2, topoGraph);
                }
                //                    std::cout << "infeasible" << std::endl;
                /** line 5 **/
                //                    return std::make_pair(it->agentId1, it->agentId2);
                return std::make_pair(edgePair.first.dest.agentId, edgePair.second.dest.agentId);
                //                } else {
                //                    std::cout << "failed: " << it->agentId1 << " " << it->state1 << " " << it->agentId2 << ""
                //                              << it->state2 << std::endl;
                //                }
                /** unreachable code, only for execution safety  **/
                it = unsettledEdgePairs.erase(it);
                ++erasedEdges;
                /** unreachable code end **/
            } /** line 6 **/
            else if (selectedEdges.size() == 1) {
                /** line 7 **/
                //                addedEdges.emplace_back(selectedEdges[0]);
                //                boost::add_edge(selectedEdges[0].first, selectedEdges[0].second, topoGraph);
                auto &edge = selectedEdges.front();
                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                    addedEdges.emplace_back(edgePair, edge);
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                }

                /** line 8 **/
                it = unsettledEdgePairs.erase(it);
                /** line 9 **/
                ++erasedEdges;
                //                std::cout << "choose: " << selectedEdges[0].first << " " << selectedEdges[0].second << std::endl;
            } else {
                ++it;
                /*if (speed) {
                    */
                /** line 11 **/ /*
                    //                auto it = sharedNodesList.begin();
                    auto edge1 = edgePair.first;
                    auto edge2 = edgePair.second;
                    orderEdgesByStart(edge1, edge2);

                    //                    auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
                    //                    auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge1);

                    */
                /** line 12 **/ /*
                    // TODO: add randomness here
                    if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                        addedEdges.emplace_back(nodeId1, nodeId2);
                        boost::add_edge(nodeId1, nodeId2, topoGraph);
                    }

                    */
                /** line 13 **/ /*
                    it = unsettledEdgePairs.erase(it);
                    ++erasedEdges;

                    //            std::cout << "random choose: " << nodeId1 << " " << nodeId2 << std::endl;
                } else {
                    ++it;
                }*/
            }
        }

        //        std::cout << erasedEdges << std::endl;

        if (speed) {
            for (auto it = unsettledEdgePairs.begin(); it != unsettledEdgePairs.end();) {
                auto &edgePair = *it;
                auto selectedEdges = feasibilityCheckEdgePair(edgePair);

                if (selectedEdges.empty()) {
                    for (auto [_edgePair, _edge]: addedEdges) {
                        auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(_edge);
                        boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    }
                    return std::make_pair(edgePair.first.dest.agentId, edgePair.second.dest.agentId);
                } else if (selectedEdges.size() == 1) {
                    auto &edge = selectedEdges.front();
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                    if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                        addedEdges.emplace_back(edgePair, edge);
                        boost::add_edge(nodeId1, nodeId2, topoGraph);
                    }

                    /** line 8 **/
                    it = unsettledEdgePairs.erase(it);
                    /** line 9 **/
                    ++erasedEdges;
                    break;
                } else {
                    auto [edge1, edge2] = orderEdgesBySave(edgePair);
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge1);
                    if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                        addedEdges.emplace_back(edgePair, edge1);
                        boost::add_edge(nodeId1, nodeId2, topoGraph);
                    }
                    it = unsettledEdgePairs.erase(it);
                    ++erasedEdges;
                }
            }

        } else {
            /** line 10 **/
            if (erasedEdges == 0 && !unsettledEdgePairs.empty()) {
                /** line 11 **/
                auto it = unsettledEdgePairs.begin();
                auto edgePair = *it;
                auto [edge1, edge2] = orderEdgesBySave(edgePair);

                //            auto nodeId1 = pathTopoNodeIds[it->agentId1][it->state1 + 1];
                //            auto nodeId2 = pathTopoNodeIds[it->agentId2][it->state2];
                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge1);

                /** line 12 **/
                // TODO: add randomness here
                bool recursiveEdgeAdded = !isEdgeInTopoGraph(nodeId1, nodeId2);
                if (recursiveEdgeAdded) {
                    addedEdges.emplace_back(edgePair, edge1);
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                }

                /** line 13 **/
                unsettledEdgePairs.erase(it);

                /** exhaustive version of Algorithm 1 **/
                if (recursive) {
                    auto unsettledEdgePairsBackup = unsettledEdgePairs;
                    //                std::cout << "recursive" << std::endl;
                    auto result = feasibilityCheckHelper(unsettledEdgePairsBackup, recursive);
                    if (firstAgentArrivingTimestep > 0) {
                        feasibilityCheckRecursionCount++;
                    }
                    if (recursiveEdgeAdded) {
                        boost::remove_edge(nodeId1, nodeId2, topoGraph);
                        addedEdges.pop_back();
                    }

                    if (result.first == agents.size() && result.second == agents.size()) {
                        for (auto [_edgePair, _edge]: addedEdges) {
                            auto [_nodeId1, _nodeId2] = getTopoEdgeBySDGEdge(_edge);
                            boost::remove_edge(_nodeId1, _nodeId2, topoGraph);
                        }
                        return result;
                    }
                    //                auto nodeId3 = pathTopoNodeIds[it->agentId1][it->state1];
                    //                auto nodeId4 = pathTopoNodeIds[it->agentId2][it->state2 + 1];
                    //                addedEdges.emplace_back(nodeId4, nodeId3);
                    //                boost::add_edge(nodeId4, nodeId3, topoGraph);

                    auto [nodeId3, nodeId4] = getTopoEdgeBySDGEdge(edge2);
                    if (!isEdgeInTopoGraph(nodeId3, nodeId4)) {
                        addedEdges.emplace_back(edgePair, edge2);
                        boost::add_edge(nodeId3, nodeId4, topoGraph);
                    }
                }
                //            std::cout << "random choose: " << nodeId1 << " " << nodeId2 << std::endl;
            }
        }
    }
    savedAddedEdges.clear();
    for (auto [edgePair, edge]: addedEdges) {
        savedAddedEdges.emplace(edgePair, edge);
        auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
        boost::remove_edge(nodeId1, nodeId2, topoGraph);
    }
    //    std::cout << "feasible" << std::endl;
    /** line 14 **/
    return std::make_pair(agents.size(), agents.size());
}


/*
std::pair<size_t, size_t> NodeDependencyGraph::feasibilityCheckHelperNew(
        std::list<SDGEdgePair> &unsettledEdgePairs,
        bool recursive, bool init) {
    std::vector<std::pair<SDGEdgePair, SDGEdge>> addedEdges;
    //    std::cout << "feasibility check" << std::endl;

    while (!unsettledEdgePairs.empty()) {

        if (firstAgentArrivingTimestep == 0) {
            feasibilityCheckLoopCount++;
        }

        if (!init) {
            for (auto it = unsettledEdgePairs.begin(); it != unsettledEdgePairs.end();) {
                auto edgePair = *it;
                auto selectedEdges = feasibilityCheckEdgePair(edgePair);

                if (selectedEdges.size() == 2) {
                    auto edge1 = edgePair.first;
                    auto edge2 = edgePair.second;
                    orderEdgesByStart(edge1, edge2);
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge1);
                    bool recursiveEdgeAdded = !isEdgeInTopoGraph(nodeId1, nodeId2);
                    if (recursiveEdgeAdded) {
                        addedEdges.emplace_back(edgePair, edge1);
                        boost::add_edge(nodeId1, nodeId2, topoGraph);
                    }
                    it = unsettledEdgePairs.erase(it);
                    if (recursive) {
                        auto unsettledEdgePairsBackup = unsettledEdgePairs;
                        auto result = feasibilityCheckHelperNew(unsettledEdgePairsBackup, recursive, init);
                        if (firstAgentArrivingTimestep > 0) {
                            feasibilityCheckRecursionCount++;
                        }
                        if (recursiveEdgeAdded) {
                            boost::remove_edge(nodeId1, nodeId2, topoGraph);
                            addedEdges.pop_back();
                        }
                        if (result.first == agents.size() && result.second == agents.size()) {
                            for (auto [_edgePair, _edge]: addedEdges) {
                                auto [_nodeId1, _nodeId2] = getTopoEdgeBySDGEdge(_edge);
                                boost::remove_edge(_nodeId1, _nodeId2, topoGraph);
                            }
                            return result;
                        }
                        auto [nodeId3, nodeId4] = getTopoEdgeBySDGEdge(edgePair.second);
                        if (!isEdgeInTopoGraph(nodeId3, nodeId4)) {
                            addedEdges.emplace_back(edgePair, edge2);
                            boost::add_edge(nodeId3, nodeId4, topoGraph);
                        }
                    }
                } else
                    break;
            }
        }

        for (auto it = unsettledEdgePairs.begin(); it != unsettledEdgePairs.end();) {
            auto &edgePair = *it;

            if (firstAgentArrivingTimestep == 0) {
                feasibilityCheckTopoCount++;
            }
            auto selectedEdges = feasibilityCheckEdgePair(edgePair);
            if (selectedEdges.empty()) {
                for (auto [nodeId1, nodeId2]: addedEdges) {
                    boost::remove_edge(nodeId1, nodeId2, topoGraph);
                }
                return std::make_pair(edgePair.first.dest.agentId, edgePair.second.dest.agentId);
            } else if (selectedEdges.size() == 1) {
                auto &edge = selectedEdges.front();
                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                    addedEdges.emplace_back(nodeId1, nodeId2);
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                }
                it = unsettledEdgePairs.erase(it);
            } else {
                ++it;
            }
        }

        init = false;
    }
    savedAddedEdges.clear();
    for (auto [nodeId1, nodeId2]: addedEdges) {
        boost::remove_edge(nodeId1, nodeId2, topoGraph);
    }
    //    std::cout << "feasible" << std::endl;
    return std::make_pair(agents.size(), agents.size());
}
*/


std::pair<size_t, size_t> NodeDependencyGraph::feasibilityCheckTest(bool recursive) {
    //    std::cerr << "start feasibility check" << std::endl;

    std::list<SDGEdgePair> unsettledEdgePairs;

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
            auto edgePair = makeSDGEdgePair(
                    sharedNodePair.agentId1, sharedNodePair.state1,
                    sharedNodePair.agentId2, sharedNodePair.state2);
            unsettledEdgePairs.emplace_back(edgePair);
        }
    }

    if (firstAgentArrivingTimestep == 0) {
        feasibilityCheckUnsettledCount += unsettledEdgePairs.size();
    }

    SPDLOG_DEBUG("feasibility check: {} shared nodes", unsettledEdgePairs.size());

    if (onlineOpt) {
//        auto size = unsettledEdgePairs.size();
        auto result = feasibilityCheckHelper(unsettledEdgePairs, false, true);
        if (result.first == agents.size() && result.second == agents.size()) {
//            SPDLOG_INFO("1 {}", size);
            return result;
        }
//        SPDLOG_INFO("0 {}", size);
        unsettledEdgePairs.clear();
        for (const auto &[nodeId, sharedNode]: sharedNodes) {
            for (const auto &sharedNodePair: sharedNode) {
                auto edgePair = makeSDGEdgePair(
                        sharedNodePair.agentId1, sharedNodePair.state1,
                        sharedNodePair.agentId2, sharedNodePair.state2);
                unsettledEdgePairs.emplace_back(edgePair);
            }
        }
    }
    return feasibilityCheckHelper(unsettledEdgePairs, recursive, false);

    //    return feasibilityCheckHelperNew(unsettledEdgePairs, recursive);
}

NodeDependencyGraph::SDGEdgePair NodeDependencyGraph::orderEdgesByStart(const SDGEdgePair &edgePair) {
    auto time1 = timestamps[edgePair.first.source.agentId][edgePair.first.source.state];
    auto time2 = timestamps[edgePair.second.source.agentId][edgePair.second.source.state];
    if (time1 <= time2) return edgePair;
    return std::make_pair(edgePair.second, edgePair.first);
}

NodeDependencyGraph::SDGEdgePair NodeDependencyGraph::orderEdgesBySave(const SDGEdgePair &edgePair) {
    auto it = savedAddedEdges.find(edgePair);
    if (it == savedAddedEdges.end()) {
        return orderEdgesByStart(edgePair);
    }
    if (it->second == edgePair.first) return edgePair;
    return std::make_pair(edgePair.second, edgePair.first);
}
