//
// Created by liuyh on 9/12/2024.
//

#ifndef MAPF_DP_SESSOLVER_H
#define MAPF_DP_SESSOLVER_H

#include "Solver.h"

#include <utility>

#include <utility>
#include "../../Switchable-Edge-Search/src/Algorithm/simulator.h"

class SESSolver : public Solver {
protected:
    std::shared_ptr<ses::Simulator> simulator;
public:
    SESSolver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType, std::shared_ptr<ses::Simulator> simulator)
        : Solver(graph, agents, makeSpanType), simulator(std::move(simulator)) {}

    std::string getSolverName() override { return "ses"; }

    bool solve() override;
};


#endif//MAPF_DP_SESSOLVER_H
