//
// Created by liuyh on 29/4/2023.
//

#ifndef MAPF_DP_DISCRETEPIBTSIMULATOR_H
#define MAPF_DP_DISCRETEPIBTSIMULATOR_H

#include "DiscreteSimulator.h"
#include "PIBTSimulator.h"
#include "../../time-independent-planning/time_independent/include/problem.hpp"
#include "../../time-independent-planning/third_party/grid-pathfinding/graph/include/graph.hpp"
#include "../../time-independent-planning/time_independent/include/causal_pibt_mapf.hpp"

class DiscretePIBTSimulator : public DiscreteSimulator, public PIBTSimulator {
public:
    DiscretePIBTSimulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed) :
            Simulator(graph, agents, seed) {}

    unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep,
                 unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) override;

private:
    std::shared_ptr<pibt::Grid> G;        // graph
    std::vector<std::shared_ptr<pibt::Agent> > A;                  // all agents
    std::shared_ptr<std::mt19937> MT;          // seed
//    bool solved;               // success or not
    int max_activation = 10;  // time step limit
//    int elapsed;               // computation time (ms)
    std::vector<pibt::States> exec;

    void initPIBTVariables();

    bool checkSolved();

    void convertResult();
};


#endif //MAPF_DP_DISCRETEPIBTSIMULATOR_H
