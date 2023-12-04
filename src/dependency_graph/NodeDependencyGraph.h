//
// Created by liuyh on 4/12/2023.
//

#ifndef MAPF_DP_NODEDEPENDENCYGRAPH_H
#define MAPF_DP_NODEDEPENDENCYGRAPH_H

#include "DependencyGraph.h"


class NodeDependencyGraph : public DependencyGraph {
public:
    NodeDependencyGraph(Graph &graph, std::vector<Agent> &agents, std::vector<std::vector<unsigned int>> &paths, const double &firstAgentArrivingTimestep) : DependencyGraph(graph, agents, paths, firstAgentArrivingTimestep){};

    void init();

    SDGEdgePair makeSDGEdgePair(size_t agentId1, unsigned int state1, size_t agentId2, unsigned int state2);

    void initSharedNodes(size_t i, size_t j);

    void updateSharedNode(unsigned int nodeId, size_t agentId, unsigned int state);

    std::vector<SDGEdge> feasibilityCheckEdgePair(const SDGEdgePair &edgePair);

    std::pair<size_t, size_t> feasibilityCheckHelper(
            std::list<SDGEdgePair> &unsettledEdgePairs,
            bool recursive, bool speed = true
            //            std::vector<std::pair<unsigned int, unsigned int>> &addedEdges
    );

    std::pair<size_t, size_t> feasibilityCheckTest(bool recursive);

protected:
    void orderEdgesByStart(SDGEdge &edge1, SDGEdge &edge2);

};


#endif//MAPF_DP_NODEDEPENDENCYGRAPH_H
