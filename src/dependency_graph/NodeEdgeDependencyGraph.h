//
// Created by liuyh on 7/8/2023.
//

#ifndef MAPF_DP_NODEEDGEDEPENDENCYGRAPH_H
#define MAPF_DP_NODEEDGEDEPENDENCYGRAPH_H

#include "DependencyGraph.h"

class NodeEdgeDependencyGraph : public DependencyGraph {
public:
    struct SDGNode {
        size_t agentId;
        unsigned int state;
    };

    friend bool operator<(const SDGNode& lhs, const SDGNode& rhs) {
        if (lhs.agentId == rhs.agentId) return lhs.state < rhs.state;
        return lhs.agentId < rhs.agentId;
    }

    friend bool operator==(const SDGNode& lhs, const SDGNode& rhs) {
        return lhs.agentId == rhs.agentId && lhs.state == rhs.state;
    }

    friend std::ostream& operator<<(std::ostream& out, const SDGNode& node) {
        return out << "(" << node.agentId << "," << node.state << ")";
    }

    struct SDGEdge {
        SDGNode source, dest;
    };

    friend bool operator<(const SDGEdge& lhs, const SDGEdge& rhs) {
        return lhs.source == rhs.source ? lhs.dest < rhs.dest : lhs.source < rhs.source;
    }

    friend bool operator==(const SDGEdge& lhs, const SDGEdge& rhs) {
        return lhs.source == rhs.source && lhs.dest == rhs.dest;
    }

    friend std::ostream& operator<<(std::ostream& out, const SDGEdge& edge) {
        return out << edge.source << "->" << edge.dest;
    }

    typedef std::pair<SDGEdge, SDGEdge> SDGEdgePair;

    // each SDGNode has a corresponding SDGData
    struct SDGData {
        // conflicts between the current SDGNode and other SDGNodes
        std::vector<SDGNode> conflicts;
        // determined edges from these SDGNodes (sources) to the current SDGNode (destination)
        // this recording direction is designed for unnecessary blocking
        std::vector<SDGNode> determinedEdgeSources;
        // contain unsettled edges from the current SDGNode (source) to some other SDGNodes (destination)
        std::vector<SDGEdgePair> unsettledEdgePairs;
    };

    std::vector<std::vector<SDGData>> sdgData;
    std::set<SDGEdgePair> unsettledEdgePairsSet;
    std::vector<SharedNodePair> sharedStates;
    std::vector<SDGEdge> savedAddedEdges;

    NodeEdgeDependencyGraph(Graph &graph, std::vector<Agent> &agents, std::vector<std::vector<unsigned int>> &paths, const double &firstAgentArrivingTimestep)  : DependencyGraph(graph, agents, paths, firstAgentArrivingTimestep) {};

    void init();

    std::pair<unsigned int, unsigned int> getTopoEdgeBySDGEdge(const SDGEdge &edge) const {
        return std::make_pair(pathTopoNodeIds[edge.source.agentId][edge.source.state],
                              pathTopoNodeIds[edge.dest.agentId][edge.dest.state]);
    };

    bool isEdgeInTopoGraph(unsigned int nodeId1, unsigned int nodeId2);

    bool isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2);

    bool generateUnsettledEdges();

    void initSharedNodes(size_t i, size_t j);

    Graph::NodeEdgeState getNodeEdgeState(size_t agentId, unsigned int state);

    void initSharedStates(size_t agentId1, size_t agentId2);

    std::vector<SDGEdge> updateSharedNode(size_t agentId, unsigned int state, bool dryRun = false);

    std::pair<size_t, size_t> feasibilityCheckHelper(
            std::list<SDGEdgePair> &sharedNodesList,
            bool recursive, bool save
            //            std::vector<std::pair<unsigned int, unsigned int>> &addedEdges
    );

    std::pair<size_t, size_t> feasibilityCheckTest(bool recursive, bool save = false);

    bool singleAgentCheck(size_t agentId);

    void addSavedEdges();
};


#endif//MAPF_DP_NODEEDGEDEPENDENCYGRAPH_H
