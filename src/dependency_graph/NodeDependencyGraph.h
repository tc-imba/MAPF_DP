//
// Created by liuyh on 4/12/2023.
//

#ifndef MAPF_DP_NODEDEPENDENCYGRAPH_H
#define MAPF_DP_NODEDEPENDENCYGRAPH_H

#include "DependencyGraph.h"


class NodeDependencyGraph : public DependencyGraph {
public:
//    std::vector<SDGEdge> savedAddedEdges;
    std::map<SDGEdgePair, SDGEdge> savedAddedEdges, ignoredEdges;
    std::vector<int> component;
    int componentNum = 0;

    NodeDependencyGraph(Graph &graph, std::vector<Agent> &agents, std::vector<std::vector<unsigned int>> &paths, const double &firstAgentArrivingTimestep) : DependencyGraph(graph, agents, paths, firstAgentArrivingTimestep){};

    void init();

    SDGEdgePair makeSDGEdgePair(size_t agentId1, unsigned int state1, size_t agentId2, unsigned int state2);

    void initSharedNodes(size_t i, size_t j);

    void updateSharedNode(unsigned int nodeId, size_t agentId, unsigned int state);

    std::vector<SDGEdge> feasibilityCheckEdgePair(const SDGEdgePair &edgePair);

    std::pair<size_t, size_t> feasibilityCheckHelper(
            std::list<SDGEdgePair> &unsettledEdgePairs,
//            const std::vector<bool> &agentIgnored,
            bool recursive, bool speed = true
    );

    std::pair<size_t, size_t> feasibilityCheckHelperNew(
            std::list<SDGEdgePair> &unsettledEdgePairs,
            bool recursive, bool init = true
    );

    void groupAgents(std::list<SDGEdgePair> &unsettledEdgePairs);

    void filterUnsettledEdgePairs(std::list<SDGEdgePair> &unsettledEdgePairs);

    void orderUnsettledEdgePairs(std::list<SDGEdgePair> &unsettledEdgePairs);

    std::pair<size_t, size_t> feasibilityCheckTest(bool recursive, bool ignore = true, bool fast = false);

    void addSavedEdges();

    void removeSavedEdges();

protected:
    SDGEdgePair orderEdgesByStart(const SDGEdgePair &edgePair);

    SDGEdgePair orderEdgesBySave(const SDGEdgePair &edgePair);

};


#endif//MAPF_DP_NODEDEPENDENCYGRAPH_H
