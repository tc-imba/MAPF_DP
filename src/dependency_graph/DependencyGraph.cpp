//
// Created by liuyh on 7/8/2023.
//

#include "DependencyGraph.h"

bool DependencyGraph::isEdgeInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
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

bool DependencyGraph::isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
    auto visitor = TopoGraphBFSVisitor(nodeId2);
    try {
        boost::breadth_first_search(topoGraph, nodeId1, boost::visitor(visitor));
    } catch (...) {
        return true;
    }
    return false;
}