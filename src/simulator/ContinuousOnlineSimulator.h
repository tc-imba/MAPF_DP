//
// Created by liu on 7/1/2022.
//

#ifndef MAPF_DP_CONTINUOUSONLINESIMULATOR_H
#define MAPF_DP_CONTINUOUSONLINESIMULATOR_H

#include "ContinuousSimulator.h"
#include "OnlineSimulator.h"
#include "../dependency_graph/NodeEdgeDependencyGraph.h"

class ContinuousOnlineSimulator : public ContinuousSimulator, public OnlineSimulator {
protected:
    std::set<NodeEdgeDependencyGraph::SDGEdge> cycleCheckAddedEdges;
    NodeEdgeDependencyGraph depGraph;

public:
    bool snapshot = false;
    double deltaTimestep = 0;

    std::string removeRedundant = "none";
    std::string snapshotOrder = "none";

    ContinuousOnlineSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) : Simulator(graph, agents, seed), depGraph(graph, agents, paths, firstAgentArrivingTimestep) {}

    unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep,
                 unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;

    void initSimulation();

    void initChecks();

    void unnecessaryBlockCheck();

//    void unsharedCheck();
//
//    void neighborCheck();
//
//    void deadEndCheck();

    void singleAgentCheck();

    void naiveCycleCheckHelper(std::vector<size_t> &readyList, size_t length, size_t start, size_t current,
                               std::vector<bool> &check, std::vector<size_t> &maxReadyList);

    void naiveCycleCheck();

    void heuristicCycleCheck();

    void cycleCheck();

    std::pair<size_t, size_t> feasibilityCheck();

    void print(std::ostream &out) const override;

    void printPaths(double maxTimeStep);
};


#endif //MAPF_DP_CONTINUOUSONLINESIMULATOR_H
