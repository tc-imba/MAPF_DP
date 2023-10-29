//
// Created by liuyh on 7/8/2023.
//

#include "NodeEdgeDependencyGraph.h"
#include <boost/range/join.hpp>

void NodeEdgeDependencyGraph::init() {
    paths.clear();
    paths.resize(agents.size());
    timestamps.clear();
    timestamps.resize(agents.size());
    deadEndStates.clear();
    deadEndStates.resize(agents.size());
    pathTopoNodeIds.clear();
    pathTopoNodeIds.resize(agents.size());
    topoGraph.clear();
    sdgData.resize(agents.size());

    unsigned int topoGraphNodeNum = 0;
    for (size_t i = 0; i < agents.size(); i++) {
        for (size_t j = 0; j < solver->solution->plans[i]->path.size(); j++) {
            auto newNodeId = solver->solution->plans[i]->path[j].nodeId;
            auto newNodeTimestamp = solver->solution->plans[i]->path[j].estimatedTime;
            SPDLOG_DEBUG("{} {} {} {}", i, j, newNodeId, newNodeTimestamp);
            if (paths[i].empty() || paths[i].back() != newNodeId) {
                paths[i].emplace_back(newNodeId);
                timestamps[i].emplace_back(newNodeTimestamp);
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

    for (const auto &sharedNodePair: sharedStates) {
        // TODO: really need to duplicate?
        sdgData[sharedNodePair.agentId1][sharedNodePair.state1].conflicts.push_back(
                {sharedNodePair.agentId2, sharedNodePair.state2});
        sdgData[sharedNodePair.agentId2][sharedNodePair.state2].conflicts.push_back(
                {sharedNodePair.agentId1, sharedNodePair.state1});
    }

    generateUnsettledEdges();

    for (size_t i = 0; i < sdgData.size(); i++) {
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
    }
}

bool NodeEdgeDependencyGraph::isEdgeInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
    auto neighborEdges = boost::out_edges(nodeId1, topoGraph);
    return std::any_of(neighborEdges.first, neighborEdges.second, [&](const auto &edge) {
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

bool NodeEdgeDependencyGraph::isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
    auto visitor = TopoGraphBFSVisitor(nodeId2);
    try {
        boost::breadth_first_search(topoGraph, nodeId1, boost::visitor(visitor));
    } catch (...) {
        return true;
    }
    return false;
}

NodeEdgeDependencyGraph::SDGNodePair NodeEdgeDependencyGraph::makeSDGNodePair(size_t agentId1, unsigned int state1, size_t agentId2, unsigned int state2) {
    SDGNode node1 = {agentId1, state1};
    SDGNode node2 = {agentId2, state2};
    if (state1 % 2 == 0 && state2 % 2 == 1) {
        // node-edge conflict
        return std::make_pair(node2, node1);
    }
    if (state1 % 2 == 1 && state2 % 2 == 0) {
        // edge-node conflict
        return std::make_pair(node1, node2);
    }
    // node-node or edge-edge conflict
    if (node1 < node2) {
        return std::make_pair(node1, node2);
    } else {
        return std::make_pair(node2, node1);
    }
}

NodeEdgeDependencyGraph::SDGEdgePair NodeEdgeDependencyGraph::makeSDGEdgePair(size_t agentId1, unsigned int state1, size_t agentId2, unsigned int state2) {
    SDGEdge edge1{}, edge2{};
    //                    std::cout << agentId1 << " " << state1 << " " << agentId2 << " " << state2 << " ";
    if (state1 % 2 == 0 && state2 % 2 == 0) {
        // node-node
        edge1 = {SDGNode{agentId2, state2 + 2}, SDGNode{agentId1, state1 - 1}};
        edge2 = {SDGNode{agentId1, state1 + 2}, SDGNode{agentId2, state2 - 1}};
    } else if (state1 % 2 == 1 && state2 % 2 == 1) {
        // edge-edge
        edge1 = {SDGNode{agentId2, state2 + 1}, SDGNode{agentId1, state1}};
        edge2 = {SDGNode{agentId1, state1 + 1}, SDGNode{agentId2, state2}};
    } else if (state1 % 2 == 0 && state2 % 2 == 1) {
        // node-edge
        edge1 = {SDGNode{agentId2, state2 + 1}, SDGNode{agentId1, state1 - 1}};
        edge2 = {SDGNode{agentId1, state1 + 2}, SDGNode{agentId2, state2}};
    } else {
        // edge-node
        edge1 = {SDGNode{agentId2, state2 + 2}, SDGNode{agentId1, state1}};
        edge2 = {SDGNode{agentId1, state1 + 1}, SDGNode{agentId2, state2 - 1}};
    }

    // The next two lines are correct, but less readable than the code above
    //                     edge1 = {SDGNode{agentId2, state2 + 2 - state2 % 2}, SDGNode{agentId1, state1 - 1 + state1 % 2}};
    //                     edge2 = {SDGNode{agentId1, state1 + 2 - state1 % 2}, SDGNode{agentId2, state2 - 1 + state2 % 2}};

    // This is wrong
    /*if (state1 % 2 == 0) {
                        edge1 = {SDGNode{agentId2, state2 + 2}, SDGNode{agentId1, state1 - 1}};
                    } else {
                        edge1 = {SDGNode{agentId2, state2 + 1}, SDGNode{agentId1, state1}};
                    }
                    if (state2 % 2 == 0) {
                        edge2 = {SDGNode{agentId1, state1 + 2}, SDGNode{agentId2, state2 - 1}};
                    } else {
                        edge2 = {SDGNode{agentId1, state1 + 1}, SDGNode{agentId2, state2}};
                    }*/
    if (edge1 < edge2) {
        return std::make_pair(edge1, edge2);
    } else {
        return std::make_pair(edge2, edge1);
    }
}


bool NodeEdgeDependencyGraph::generateUnsettledEdges() {
    std::vector<SDGNodePair> nnNodePairs, neNodePairs, eeNodePairs, fixedNodePairs;

    for (size_t agentId1 = 0; agentId1 < sdgData.size(); agentId1++) {
        for (unsigned int state1 = 0; state1 < sdgData[agentId1].size(); state1++) {
            auto &sdgDataItem = sdgData[agentId1][state1];
            for (const auto &conflict: sdgDataItem.conflicts) {
                size_t agentId2 = conflict.agentId;
                unsigned int state2 = conflict.state;
                // prevent duplication
                if (agentId1 < agentId2 || (agentId1 == agentId2 && state1 < state2)) {
                    auto [edge1, edge2] = makeSDGEdgePair(agentId1, state1, agentId2, state2);
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
                    auto nodePair = makeSDGNodePair(agentId1, state1, agentId2, state2);
                    if (invalid == 0) {
                        // record the pair of unsettled edges
                        if (state1 % 2 == 0 && state2 % 2 == 0) {
                            nnNodePairs.emplace_back(nodePair);
                        } else if (state1 % 2 == 1 && state2 % 2 == 1) {
                            eeNodePairs.emplace_back(nodePair);
                        } else {
                            neNodePairs.emplace_back(nodePair);
                        }
                        //                        sdgData[edge1.dest.agentId][edge1.dest.state].unsettledEdgePairs.push_back(edgePair);
                        //                        sdgData[edge2.dest.agentId][edge2.dest.state].unsettledEdgePairs.push_back(edgePair);
                        SPDLOG_DEBUG("generate unsettled edge pair {} {} with {}-{} conflict {} {}", edge1, edge2,
                                     state1 % 2 == 0 ? "node" : "edge", state2 % 2 == 0 ? "node" : "edge",
                                     SDGNode{agentId1, state1}, conflict);
                    } else if (invalid == 1) {
                        // edge2 is determined edge
                        fixedNodePairs.emplace_back(nodePair);
                        sdgData[edge2.dest.agentId][edge2.dest.state].determinedEdgeSources.push_back(
                                {edge2.source.agentId, edge2.source.state});
                        if (debug) {
                            //                            std::cout << "generate determined edge " << edge2 << std::endl;
                        }
                    } else if (invalid == 2) {
                        // edge1 is determined edge
                        fixedNodePairs.emplace_back(nodePair);
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
    std::vector<SDGNodePair> addedNodePairs;
    numAllNodePairs = nnNodePairs.size() + neNodePairs.size() + eeNodePairs.size() + fixedNodePairs.size();
    numFixedNodePairs = fixedNodePairs.size();

    if (removeRedundant == "physical") {
        generateAddedNodePairsPhysical(nnNodePairs, neNodePairs, eeNodePairs, fixedNodePairs, addedNodePairs);
    } else if (removeRedundant == "graph") {
        generateAddedNodePairsGraph(nnNodePairs, neNodePairs, eeNodePairs, fixedNodePairs, addedNodePairs);
    } else {
        generateAddedNodePairsNone(nnNodePairs, neNodePairs, eeNodePairs, fixedNodePairs, addedNodePairs);
    }

    numAddedNodePairs = addedNodePairs.size();
//    std::cerr << "all: " << numAllNodePairs << " fixed: " << numFixedNodePairs << " added: " << numAddedNodePairs << std::endl;

    // all conflicts after removing redundant
    for (const auto &nodePair: addedNodePairs) {
        auto edgePair = makeSDGEdgePair(nodePair.first.agentId, nodePair.first.state, nodePair.second.agentId, nodePair.second.state);
        const auto &[edge1, edge2] = edgePair;
        sdgData[edge1.dest.agentId][edge1.dest.state].unsettledEdgePairs.push_back(edgePair);
        sdgData[edge2.dest.agentId][edge2.dest.state].unsettledEdgePairs.push_back(edgePair);
    }
    return true;
}


void NodeEdgeDependencyGraph::removeRedundantNodePair(std::set<SDGNodePair> &nodePairs, const NodeEdgeDependencyGraph::SDGNodePair &nodePair, int dState1, int dState2) {
    auto newNodePair = makeSDGNodePair(nodePair.first.agentId, nodePair.first.state + dState1, nodePair.second.agentId, nodePair.second.state + dState2);
    auto it = nodePairs.find(newNodePair);
    if (it != nodePairs.end()) {
        nodePairs.erase(it);
        if (debug) {
            auto edgePair = makeSDGEdgePair(newNodePair.first.agentId, newNodePair.first.state, newNodePair.second.agentId, newNodePair.second.state);
            const auto &[edge1, edge2] = edgePair;
            SPDLOG_DEBUG("remove redundant unsettled edge pair {} {} with {}-{} conflict {} {}", edge1, edge2,
                         newNodePair.first.state % 2 == 0 ? "node" : "edge", newNodePair.second.state % 2 == 0 ? "node" : "edge",
                         newNodePair.first, newNodePair.second);
        }
    }
}

void NodeEdgeDependencyGraph::generateAddedNodePairsNone(
        const std::vector<SDGNodePair> &nnNodePairs,
        const std::vector<SDGNodePair> &neNodePairs,
        const std::vector<SDGNodePair> &eeNodePairs,
        const std::vector<SDGNodePair> &fixedNodePairs,
        std::vector<SDGNodePair> &addedNodePairs) {
    for (const auto &nodePair: nnNodePairs) {
        addedNodePairs.emplace_back(nodePair);
    }
    for (const auto &nodePair: neNodePairs) {
        addedNodePairs.emplace_back(nodePair);
    }
    for (const auto &nodePair: eeNodePairs) {
        addedNodePairs.emplace_back(nodePair);
    }
}

void NodeEdgeDependencyGraph::generateAddedNodePairsPhysical(
        const std::vector<SDGNodePair> &nnNodePairs,
        const std::vector<SDGNodePair> &neNodePairs,
        const std::vector<SDGNodePair> &eeNodePairs,
        const std::vector<SDGNodePair> &fixedNodePairs,
        std::vector<SDGNodePair> &addedNodePairs) {

    std::set<SDGNodePair> neNodePairsSet(neNodePairs.begin(), neNodePairs.end());
    std::set<SDGNodePair> eeNodePairsSet(eeNodePairs.begin(), eeNodePairs.end());

    // node-node conflict
    for (const auto &nodePair: nnNodePairs) {
        addedNodePairs.emplace_back(nodePair);
        // suppose nodePair = <v_{i,j}, v_{i',j'}>
        // remove redundant edge-edge conflicts
        removeRedundantNodePair(eeNodePairsSet, nodePair, 1, 1);  // <e_{i,j}, e_{i',j'}>
        removeRedundantNodePair(eeNodePairsSet, nodePair, -1, -1);// <e_{i,j-1}, e_{i',j'-1}>
        removeRedundantNodePair(eeNodePairsSet, nodePair, 1, -1); // <e_{i,j}, e_{i',j'-1}>
        removeRedundantNodePair(eeNodePairsSet, nodePair, -1, 1); // <e_{i,j-1}, e_{i',j'}>
        // remove redundant node-edge conflicts
        removeRedundantNodePair(neNodePairsSet, nodePair, 1, 0); // <e_{i,j}, v_{i',j'}>
        removeRedundantNodePair(neNodePairsSet, nodePair, -1, 0);// <e_{i,j-1}, v_{i',j'}>
        removeRedundantNodePair(neNodePairsSet, nodePair, 0, 1); // <v_{i,j}, e_{i',j'}>
        removeRedundantNodePair(neNodePairsSet, nodePair, 0, -1);// <v_{i,j}, v_{i',j'-1}>
    }
    // node-edge conflict
    for (const auto &nodePair: neNodePairsSet) {
        addedNodePairs.emplace_back(nodePair);
        // suppose nodePair = <e_{i,j}, v_{i',j'}>
        // we ensure the edge-node order in makeSDGNodePair
        // remove redundant edge-edge conflicts
        removeRedundantNodePair(eeNodePairsSet, nodePair, 0, 1); // <e_{i,j}, e_{i',j'}>
        removeRedundantNodePair(eeNodePairsSet, nodePair, 0, -1);// <e_{i,j}, e_{i',j'-1}>
    }
    // edge-edge conflict
    for (const auto &nodePair: eeNodePairsSet) {
        addedNodePairs.emplace_back(nodePair);
    }
}

void NodeEdgeDependencyGraph::generateAddedNodePairsGraph(
        const std::vector<SDGNodePair> &nnNodePairs,
        const std::vector<SDGNodePair> &neNodePairs,
        const std::vector<SDGNodePair> &eeNodePairs,
        const std::vector<SDGNodePair> &fixedNodePairs,
        std::vector<SDGNodePair> &addedNodePairs) {

    std::set<SDGNodePair> fixedNodePairsSet(fixedNodePairs.begin(), fixedNodePairs.end());
    std::vector<std::vector<std::vector<SDGNodePair>>> agentPairNodePairs(agents.size(), std::vector<std::vector<SDGNodePair>>(agents.size()));
    for (const auto &nodePair: boost::join(boost::join(nnNodePairs, neNodePairs), boost::join(eeNodePairs, fixedNodePairs))) {
        agentPairNodePairs[nodePair.first.agentId][nodePair.second.agentId].emplace_back(nodePair);
        agentPairNodePairs[nodePair.second.agentId][nodePair.first.agentId].emplace_back(nodePair);
    }
    for (unsigned int i = 0; i < agents.size(); i++) {
        for (unsigned int j = i + 1; j < agents.size(); j++) {
            auto &nodePairs = agentPairNodePairs[i][j];
            std::vector<SDGEdgePair> edgePairs;
            for (const auto &nodePair: nodePairs) {
                auto edgePair = makeSDGEdgePair(nodePair.first.agentId, nodePair.first.state, nodePair.second.agentId, nodePair.second.state);
                edgePairs.emplace_back(edgePair);
            }
            for (unsigned int m = 0; m < nodePairs.size(); m++) {
                bool redundant = false;
                for (unsigned int n = 0; n < nodePairs.size(); n++) {
                    if (m == n) continue;
                    if (compareSDGEdgePairWithPartialOrder(edgePairs[m], edgePairs[n])) {
                        // if C_n exists so that C_m <= C_n (partial order), then C_m is redundant
                        redundant = true;
                        break;
                    }
                }
                if (!redundant) {
                    auto it = fixedNodePairsSet.find(nodePairs[m]);
                    if (it == fixedNodePairsSet.end()) {
                        addedNodePairs.emplace_back(nodePairs[m]);
                    }
                }
            }
        }
    }
}

void NodeEdgeDependencyGraph::initSharedNodes(size_t i, size_t j) {
    std::unordered_map<unsigned int, std::vector<std::pair<size_t, unsigned int>>> m;
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


Graph::NodeEdgeState NodeEdgeDependencyGraph::getNodeEdgeState(size_t agentId, unsigned int state) {
    if (state % 2 == 0) {
        return Graph::NodeEdgeState{true, sqrt(2) / 4, paths[agentId][state / 2], 0};
    } else {
        return Graph::NodeEdgeState{false, sqrt(2) / 4, paths[agentId][state / 2], paths[agentId][state / 2 + 1]};
    }
}

void NodeEdgeDependencyGraph::initSharedStates(size_t agentId1, size_t agentId2) {
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


std::vector<NodeEdgeDependencyGraph::SDGEdge>
NodeEdgeDependencyGraph::updateSharedNode(size_t agentId, unsigned int state, bool dryRun) {
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
                SPDLOG_DEBUG("select unsettled edge pair: {} {} -> {}", pair.first, pair.second, selectedEdge);
            }
            auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(selectedEdge);
            bool alreadyAdded = isEdgeInTopoGraph(nodeId1, nodeId2);
            if (alreadyAdded) {
                SPDLOG_DEBUG("unsettled edge already added: {}", selectedEdge);
            } else {
                result.emplace_back(selectedEdge);
            }
            if (isPathInTopoGraph(nodeId2, nodeId1)) {
                SPDLOG_ERROR("error: path exists in topo graph!");
                //                    exit(-1);
            }
            if (!dryRun) {
                if (!alreadyAdded) {
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                }
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


/** Algorithm 1 **/
std::pair<size_t, size_t> NodeEdgeDependencyGraph::feasibilityCheckHelper(
        std::list<SDGEdgePair> &unsettledEdgePairs,
        bool recursive, bool save) {
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
                SPDLOG_DEBUG("no edge can be selected: {} {} {} {}", it->first, edge1Settled, it->second, edge2Settled);
                for (auto &edge: addedEdges) {
                    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                    boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    SPDLOG_DEBUG("temporarily remove settled edge: {}", edge);
                }
                return std::make_pair(it->first.dest.agentId, it->second.dest.agentId);
            } else if (selectedEdges.size() == 1) {
                auto &edge = selectedEdges[0];
                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
                if (!isEdgeInTopoGraph(nodeId1, nodeId2)) {
                    addedEdges.emplace_back(edge);
                    boost::add_edge(nodeId1, nodeId2, topoGraph);
                    SPDLOG_DEBUG("temporarily select unsettled edge pair: {} {} -> {}", it->first, it->second, edge);
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

            auto edge1 = it->first;
            auto edge2 = it->second;
            if (snapshotOrder == "start") {
                orderEdgesByStart(edge1, edge2);
            } else if (snapshotOrder == "end") {
                orderEdgesByEnd(edge1, edge2);
            } else if (snapshotOrder == "collision") {
                orderEdgesByCollision(edge1, edge2);
            }


            auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge1);

            /** line 12 **/
            // TODO: add randomness here
            bool randomEdgeSelected = !isEdgeInTopoGraph(nodeId1, nodeId2);
            if (randomEdgeSelected) {
                addedEdges.emplace_back(edge1);
                boost::add_edge(nodeId1, nodeId2, topoGraph);
                SPDLOG_DEBUG("temporarily traverse unsettled edge pair: {} {} -> {}", edge1, edge2, edge1);
            }


            /** line 13 **/
            unsettledEdgePairs.erase(it);

            /** exhaustive version of Algorithm 1 **/
            if (recursive) {
                auto sharedNodesListBackup = unsettledEdgePairs;
                //                std::cout << "recursive" << std::endl;
                auto result = feasibilityCheckHelper(sharedNodesListBackup, recursive, save);
                if (firstAgentArrivingTimestep == 0) {
                    feasibilityCheckRecursionCount++;
                }
                if (randomEdgeSelected) {
                    boost::remove_edge(nodeId1, nodeId2, topoGraph);
                    SPDLOG_DEBUG("temporarily remove settled edge: {}", edge1);
                    addedEdges.pop_back();
                }

                if (result.first == agents.size() && result.second == agents.size()) {
                    for (auto &edge: addedEdges) {
                        auto [_nodeId1, _nodeId2] = getTopoEdgeBySDGEdge(edge);
                        boost::remove_edge(_nodeId1, _nodeId2, topoGraph);
                        SPDLOG_DEBUG("temporarily remove settled edge: {}", edge);
                    }
                    if (save) {
                        savedAddedEdges.insert(savedAddedEdges.end(), addedEdges.begin(), addedEdges.end());
                    }
                    return result;
                }

                auto [nodeId3, nodeId4] = getTopoEdgeBySDGEdge(edge2);
                if (!isEdgeInTopoGraph(nodeId3, nodeId4)) {
                    addedEdges.emplace_back(edge2);
                    boost::add_edge(nodeId3, nodeId4, topoGraph);
                    SPDLOG_DEBUG("temporarily traverse unsettled edge pair: {} {} -> {}", edge1, edge2, edge2);
                }
            }
            //            std::cout << "random choose: " << nodeId1 << " " << nodeId2 << std::endl;
        }
    }
    for (auto &edge: addedEdges) {
        auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
        boost::remove_edge(nodeId1, nodeId2, topoGraph);
        SPDLOG_DEBUG("temporarily remove settled edge: {}", edge);
    }
    if (save) {
        savedAddedEdges.insert(savedAddedEdges.end(), addedEdges.begin(), addedEdges.end());
    }
    //    std::cout << "feasible" << std::endl;
    /** line 14 **/
    return std::make_pair(agents.size(), agents.size());
}


std::pair<size_t, size_t> NodeEdgeDependencyGraph::feasibilityCheckTest(bool recursive, bool save) {
    //    std::cerr << "start feasibility check" << std::endl;

    //    std::list<SharedNodePair> sharedNodesList;
    savedAddedEdges.clear();
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

    SPDLOG_DEBUG("begin feasibility check, unsettled size: {}", unsettledEdgePairs.size());

    if (firstAgentArrivingTimestep == 0) {
        feasibilityCheckUnsettledCount += unsettledEdgePairs.size();
    }

    //    return feasibilityCheckHelper(sharedNodesList, recursive);
    //    for (const auto &[nodeId, sharedNode]: sharedNodes) {
    //        for (const auto &sharedNodePair: sharedNode) {
    //            sharedNodesList.emplace_back(sharedNodePair);
    //        }
    //    }

    return feasibilityCheckHelper(unsettledEdgePairs, recursive, save);

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


bool NodeEdgeDependencyGraph::singleAgentCheck(size_t i) {
    if (agents[i].state + 1 >= pathTopoNodeIds[i].size() || agents[i].state % 2 == 1) {
        // should not happen?
        return false;
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
            NodeEdgeDependencyGraph::SDGEdge edge = {{i, agents[i].state + 2},
                                                     {j, agents[j].state + 1}};
            //                std::cout << "test:" << edge << std::endl;
            auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
            if (isPathInTopoGraph(nodeId2, nodeId1)) {
                return false;
            }
            // we can also do a feasibility check here!
        }
        //            for (const auto &edge: addedEdges) {
        //                auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
        //                boost::remove_edge(nodeId1, nodeId2, topoGraph);
        //            }
    }
    return true;
}

void NodeEdgeDependencyGraph::addSavedEdges() {
    for (auto &edge: savedAddedEdges) {
        auto [_nodeId1, _nodeId2] = getTopoEdgeBySDGEdge(edge);
        boost::add_edge(_nodeId1, _nodeId2, topoGraph);
        SPDLOG_DEBUG("add saved settled edge: {}", edge);
    }
}


void NodeEdgeDependencyGraph::orderEdgesByStart(SDGEdge &edge1, SDGEdge &edge2) {
    auto time1 = timestamps[edge1.source.agentId][edge1.source.state / 2];
    auto time2 = timestamps[edge2.source.agentId][edge2.source.state / 2];
    if (time1 > time2) std::swap(edge1, edge2);
}

void NodeEdgeDependencyGraph::orderEdgesByEnd(SDGEdge &edge1, SDGEdge &edge2) {
    auto time1 = timestamps[edge1.source.agentId][(edge1.source.state / 2 + 1 < timestamps[edge1.source.agentId].size()) ? (edge1.source.state / 2 + 1) : (edge1.source.state / 2)];
    auto time2 = timestamps[edge2.source.agentId][(edge2.source.state / 2 + 1 < timestamps[edge2.source.agentId].size()) ? (edge2.source.state / 2 + 1) : (edge2.source.state / 2)];
    if (time1 > time2) std::swap(edge1, edge2);
}

void NodeEdgeDependencyGraph::orderEdgesByCollision(SDGEdge &edge1, SDGEdge &edge2) {
}
