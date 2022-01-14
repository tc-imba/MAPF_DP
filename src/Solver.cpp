//
// Created by liu on 27/4/2021.
//

#include "Solver.h"
#include <random>
#include <iostream>

void Solver::init(MakeSpanType makeSpanType) {
    this->makeSpanType = makeSpanType;
}

Solver::Solver(Graph &graph, std::vector<Agent> &agents) : graph(graph), agents(agents) {

}


size_t Solver::combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int seed) {
    size_t result = 0;
    boost::hash_combine(result, nodeId1 ^ nodeId2);
    boost::hash_combine(result, timestep);
    boost::hash_combine(result, seed);
    return result;
}

