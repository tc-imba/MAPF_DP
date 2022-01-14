//
// Created by liu on 3/1/2022.
//

#ifndef MAPF_DP_SIMULATOR_H
#define MAPF_DP_SIMULATOR_H

#include "Solver.h"

class Simulator {
protected:
    Graph &graph;
    std::vector<Agent> agents;
    unsigned int seed;

    CBSNodePtr solution = nullptr;
    bool debug = false;

public:
    Simulator(Graph &graph, const std::vector<Agent> &agents, unsigned int seed)
            : graph(graph), agents(agents), seed(seed) {}

    void setSolution(CBSNodePtr _solution) {
        solution = std::move(_solution);
    }

    size_t combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int _seed) {
        size_t result = 0;
        boost::hash_combine(result, nodeId1 ^ nodeId2);
        boost::hash_combine(result, timestep);
        boost::hash_combine(result, _seed);
        return result;
    }

    double averageMakeSpan(MakeSpanType makeSpanType) {
        double result = 0;
        for (const auto &agent: agents) {
            if (makeSpanType == MakeSpanType::MAXIMUM) {
                result = std::max(result, (double) agent.timestep);
            } else if (makeSpanType == MakeSpanType::AVERAGE) {
                result += agent.timestep / (double) agents.size();
            }
        }
        return result;
    }

    virtual bool simulate(unsigned int &currentTimestep, unsigned int maxTimeStep) = 0;

};

#endif //MAPF_DP_SIMULATOR_H
