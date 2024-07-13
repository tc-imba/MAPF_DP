//
// Created by liuyh on 13/7/2024.
//

#ifndef MAPF_DP_ARRAYTOPOGRAPH_H
#define MAPF_DP_ARRAYTOPOGRAPH_H

#include "TopoGraph.h"

class ArrayTopoGraph : public TopoGraph {
protected:
    // connectedGraph[k][i,j] = h means that
    // staring from l_{i,j}, the smallest reachable state l_{k,h}
    std::vector<std::vector<unsigned int> > connectedGraph;
public:
    ArrayTopoGraph(DependencyGraph &depGraph): TopoGraph(depGraph) {}

    void init() override;

    void reset() override;

    bool addEdge(const DependencyGraph::SDGEdge &edge) override;

    bool removeEdge(const DependencyGraph::SDGEdge &edge) override;

    bool hasReversedPath(const DependencyGraph::SDGEdge &edge) override;

    bool hasEdge(const DependencyGraph::SDGEdge &edge) override;
};


#endif//MAPF_DP_ARRAYTOPOGRAPH_H
