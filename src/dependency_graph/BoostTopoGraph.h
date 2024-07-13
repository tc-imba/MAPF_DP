//
// Created by liuyh on 13/7/2024.
//

#ifndef MAPF_DP_BOOSTTOPOGRAPH_H
#define MAPF_DP_BOOSTTOPOGRAPH_H

#include "TopoGraph.h"

class BoostTopoGraph : public TopoGraph {
protected:
    Graph::topo_graph_t topoGraph;

public:
    BoostTopoGraph(DependencyGraph &depGraph): TopoGraph(depGraph) {}

    void init() override;

    void reset() override;

    bool addEdge(const DependencyGraph::SDGEdge &edge) override;

    bool removeEdge(const DependencyGraph::SDGEdge &edge) override;

    bool hasReversedPath(const DependencyGraph::SDGEdge &edge) override;

    bool hasEdge(const DependencyGraph::SDGEdge &edge) override;
};


#endif//MAPF_DP_BOOSTTOPOGRAPH_H
