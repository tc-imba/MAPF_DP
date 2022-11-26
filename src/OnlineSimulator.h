//
// Created by liu on 7/1/2022.
//

#ifndef MAPF_DP_ONLINESIMULATOR_H
#define MAPF_DP_ONLINESIMULATOR_H

#include "Simulator.h"

class OnlineSimulator : public Simulator {
public:
    OnlineSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed) {}

    int simulate(unsigned int &currentTimestep, unsigned int maxTimeStep,
                 unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;

    void print(std::ostream &out) const override;

    bool isHeuristicFeasibilityCheck = true;
    bool isHeuristicCycleCheck = true;
    bool isOnlyCycleCheck = false;
    bool isFeasibilityType = false;

    double executionTime = 0;
    size_t feasibilityCheckCount = 0;
    size_t feasibilityCheckTypes[4] = {0, 0, 0, 0};
    size_t cycleCheckCount = 0;
    size_t cycleCheckAgents = 0;
    size_t unblockedAgents = 0;
    size_t firstAgentArrivingTimestep = 0;
//    size_t feasibilityCheckIteration[2] = {0, 0};
//    size_t feasibilityCheckIterationTemp = 0;

private:
    struct SharedNodePair {
        size_t agentId1;
        unsigned int state1;
        size_t agentId2;
        unsigned int state2;
    };

    struct SDGNode {
        size_t agentId;
        unsigned int state;
    };

    struct SDGEdge {
        SDGNode source, dest;
    };

    typedef std::pair<SDGEdge, SDGEdge> SDGEdgePair;

    // each SDGNode has a corresponding SDGData
    struct SDGData {
        // conflicts between the current SDGNode and other SDGNodes
        std::vector<SDGNode> conflicts;
        // determined edges from these SDGNodes (sources) to the current SDGNode (destination)
        // this recording direction is designed for unnecessary blocking
        std::vector<SDGNode> determinedEdgeSources;
        //
        std::vector<SDGEdgePair> unsettledEdgePairs;
    };

    std::vector<std::vector<SDGData>> sdgData;

    std::vector<std::vector<unsigned int>> paths;
    std::vector<std::vector<std::pair<size_t, unsigned int> > > deadEndStates;
    std::unordered_map<unsigned int, std::vector<SharedNodePair>> sharedNodes;

    std::vector<std::vector<unsigned int>> pathTopoNodeIds;
    Graph::topo_graph_t topoGraph;

    std::unordered_map<size_t, size_t> nodeAgentMap;
    std::set<size_t> blocked, unblocked, moved, ready, unshared, keepMoving;

    void printSets(const std::string &title);

    void initSharedNodes(size_t i, size_t j);

    void updateSharedNode(unsigned int nodeId, size_t agentId, unsigned int state);

    void initSimulation();

    void initChecks();

    void unnecessaryBlockCheck();

    void unsharedCheck();

    void neighborCheck();

    void deadEndCheck();

    void naiveCycleCheckHelper(std::vector<size_t> &readyList, size_t length, size_t start, size_t current,
                               std::vector<bool> &check, std::vector<size_t> &maxReadyList);

    void naiveCycleCheck();

    void heuristicCycleCheck();

    void cycleCheck();

    std::pair<size_t, size_t> feasibilityCheckHelper(
            std::list<SDGEdgePair> &sharedNodesList,
            bool recursive
//            std::vector<std::pair<unsigned int, unsigned int>> &addedEdges
    );

    std::pair<size_t, size_t> feasibilityCheckTest(bool recursive);

    std::pair<size_t, size_t> feasibilityCheck();

    bool generateUnsettledEdges();


};


#endif //MAPF_DP_ONLINESIMULATOR_H
