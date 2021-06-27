//
// Created by liu on 27/4/2021.
//

#ifndef MAPF_DP_SOLVER_H
#define MAPF_DP_SOLVER_H

#include "Graph.h"

class Solver {
public:
    enum class MakeSpanType {
        MAXIMUM,
        AVERAGE,
    };
protected:
    struct Agent {
        unsigned int start, goal, current;
        unsigned int state;
        unsigned int timestep;
    };

    Graph &graph;
    std::vector<Agent> agents;
    MakeSpanType makeSpanType;

public:
    bool success = false;
    bool debug = false;
    bool allConstraint = false;

    Solver() = delete;

    Solver(Graph &graph, unsigned int agentNum);

    void generateRandomAgents(size_t seed = 0);

    void generateWarehouseAgents(size_t seed = 0, bool swap = false);

    double averageMakeSpan();

    static size_t combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int seed);

    void init(MakeSpanType makeSpanType = MakeSpanType::MAXIMUM);

    virtual bool step() = 0;
};


#endif //MAPF_DP_SOLVER_H
