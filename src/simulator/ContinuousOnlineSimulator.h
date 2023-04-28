//
// Created by liu on 7/1/2022.
//

#ifndef MAPF_DP_CONTINUOUSONLINESIMULATOR_H
#define MAPF_DP_CONTINUOUSONLINESIMULATOR_H

#include "ContinuousSimulator.h"
#include "OnlineSimulator.h"

class ContinuousOnlineSimulator : public ContinuousSimulator, public OnlineSimulator {
public:
    ContinuousOnlineSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed) {}

    int simulate(double &currentTimestep, unsigned int maxTimeStep,
                 unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;


private:
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
        return lhs.source < rhs.source;
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


//    unsigned int getNextNodeState(unsigned int state) const { return (state / 2 + 1) * 2; }

    std::pair<unsigned int, unsigned int> getTopoEdgeBySDGEdge(const SDGEdge &edge) const {
        return std::make_pair(pathTopoNodeIds[edge.source.agentId][edge.source.state],
                              pathTopoNodeIds[edge.dest.agentId][edge.dest.state]);
    };

    void initSharedNodes(size_t i, size_t j);

    void updateSharedNode(unsigned int nodeId, size_t agentId, unsigned int state);

    void initSimulation();

    void initChecks();

    void unnecessaryBlockCheck();

    void unsharedCheck();

    void neighborCheck();

    void deadEndCheck();

    void singleAgentCheck();

    void naiveCycleCheckHelper(std::vector<size_t> &readyList, size_t length, size_t start, size_t current,
                               std::vector<bool> &check, std::vector<size_t> &maxReadyList);

    void naiveCycleCheck();

    void heuristicCycleCheck();

    void cycleCheck();

    bool isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2);

    std::pair<size_t, size_t> feasibilityCheckHelper(
            std::list<SDGEdgePair> &sharedNodesList,
            bool recursive
//            std::vector<std::pair<unsigned int, unsigned int>> &addedEdges
    );

    std::pair<size_t, size_t> feasibilityCheckTest(bool recursive);

    std::pair<size_t, size_t> feasibilityCheck();

    bool generateUnsettledEdges();


};


#endif //MAPF_DP_CONTINUOUSONLINESIMULATOR_H
