//
// Created by liuyh on 13/7/2024.
//

#include "BoostTopoGraph.h"

void BoostTopoGraph::init() {
    for (size_t i = 0; i < depGraph.agents.size(); i++) {
        for (size_t j = 0; j < depGraph.pathTopoNodeIds[i].size() - 1; j++) {
            boost::add_edge(depGraph.pathTopoNodeIds[i][j], depGraph.pathTopoNodeIds[i][j + 1], topoGraph);
        }
    }
    topoGraphNodeNum = boost::num_vertices(topoGraph);
}

void BoostTopoGraph::reset() {

}

bool BoostTopoGraph::addEdge(const DependencyGraph::SDGEdge &edge) {
    if (hasEdge(edge)) { return false; }
    auto [nodeId1, nodeId2] = depGraph.getTopoEdgeBySDGEdge(edge);
    boost::add_edge(nodeId1, nodeId2, topoGraph);
    return true;
}

bool BoostTopoGraph::removeEdge(const DependencyGraph::SDGEdge &edge) {
    auto [nodeId1, nodeId2] = depGraph.getTopoEdgeBySDGEdge(edge);
    boost::remove_edge(nodeId1, nodeId2, topoGraph);
    return true;
}
bool BoostTopoGraph::hasReversedPath(const DependencyGraph::SDGEdge &edge) {
    // note: nodeId1 and nodeId1 are reversed here
    auto [nodeId2, nodeId1] = depGraph.getTopoEdgeBySDGEdge(edge);
    auto visitor = TopoGraphBFSVisitor(nodeId2);
    try {
        boost::breadth_first_search(topoGraph, nodeId1, boost::visitor(visitor));
    } catch (...) { return true; }
    return false;
}

bool BoostTopoGraph::hasEdge(const DependencyGraph::SDGEdge &edge) {
    auto [nodeId1, nodeId2] = depGraph.getTopoEdgeBySDGEdge(edge);
    auto neighborEdges = boost::out_edges(nodeId1, topoGraph);
    return std::any_of(neighborEdges.first, neighborEdges.second,
                       [&](const auto &edge) { return edge.m_target == nodeId2; });
}
