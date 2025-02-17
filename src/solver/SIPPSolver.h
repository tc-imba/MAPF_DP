//
// Created by liu on 16/2/2025.
//

#ifndef SIPPSOLVER_H
#define SIPPSOLVER_H

#include "Solver.h"
#include "IndividualSolver.h"

class SIPPSolver : IndividualSolver {
public:
    struct Node : IndividualSolver::Node {

    };

    std::shared_ptr<Agent> agent;

    SIPPSolver(std::shared_ptr<Agent> agent) : agent(std::move(agent)) {}



};


#endif//SIPPSOLVER_H
