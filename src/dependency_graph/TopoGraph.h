//
// Created by liuyh on 13/7/2024.
//

#ifndef MAPF_DP_TOPOGRAPH_H
#define MAPF_DP_TOPOGRAPH_H

#include "DependencyGraph.h"

class TopoGraph {
protected:
    DependencyGraph &depGraph;
    unsigned int topoGraphNodeNum = 0;

public:
    explicit TopoGraph(DependencyGraph &depGraph): depGraph(depGraph) {}

    virtual ~TopoGraph() = default;

    virtual void init() = 0;

    virtual void reset() = 0;

    virtual bool addEdge(const DependencyGraph::SDGEdge &edge) = 0;

    virtual bool removeEdge(const DependencyGraph::SDGEdge &edge) = 0;

    virtual bool hasReversedPath(const DependencyGraph::SDGEdge &edge) = 0;

    virtual bool hasEdge(const DependencyGraph::SDGEdge &edge) = 0;
};


#endif//MAPF_DP_TOPOGRAPH_H
