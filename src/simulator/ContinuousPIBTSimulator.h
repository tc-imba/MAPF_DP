//
// Created by liuyh on 5/4/2024.
//

#ifndef MAPF_DP_CONTINUOUSPIBTSIMULATOR_H
#define MAPF_DP_CONTINUOUSPIBTSIMULATOR_H


#include "../solver/Solver.h"
#include "ContinuousSimulator.h"
#include "PIBTSimulator.h"

class ContinuousPIBTSimulator : public ContinuousSimulator, public PIBTSimulator {
public:
    ContinuousPIBTSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed)
        : Simulator(graph, agents, seed) {}

    unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep, unsigned int delayStart = INT_MAX,
                          unsigned int delayInterval = INT_MAX) override;

    std::vector<std::vector<double>> distances;
    std::vector<std::vector<Label>> plans;

private:
    bool PIBT(unsigned int agentId);

    void calculateDistances();

    bool isConflict(double t1, const Graph::Node &srcNode1, const Graph::Node &destNode1, double t2, const Graph::Node &srcNode2, const Graph::Node &destNode2);

//    bool isConflict();
//
//    bool isConflict(const Graph::Edge &edge1, double t1, const Graph::Edge &edge2, double t2);
//
//    bool isConflict(const Graph::Edge &edge1, double t1, const Graph::Node &node2, double t2);
};


#endif//MAPF_DP_CONTINUOUSPIBTSIMULATOR_H
