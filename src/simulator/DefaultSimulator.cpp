//
// Created by liuyh on 25/4/2023.
//

#include "DefaultSimulator.h"

void DefaultSimulator::writeSimulationOutput() {
    Simulator::writeSimulationOutput();
    resultJson["partial_replan_count"] = partialReplanCount;
    resultJson["partial_replan_time"] = partialReplanTime;
    resultJson["full_replan_count"] = fullReplanCount;
    resultJson["full_replan_time"] = fullReplanTime;
}


void DefaultSimulator::print(std::ostream &out) const {
    out << executionTime << ","
        << firstAgentArrivingExecutionTime << ","
        << firstAgentArrivingTimestep << ","
        << partialReplanCount << ","
        << partialReplanTime << ","
        << fullReplanCount << ","
        << fullReplanTime;
}

void DefaultSimulator::printState(std::ostream &os, size_t i, unsigned int state) {
    auto currentNodeId = solver->solution->plans[i]->path[state].nodeId;
    os << "(" << state << "," << currentNodeId << ")";
}

double DefaultSimulator::replan() {
/*
        for (unsigned int i = 0; i < agents.size(); i++) {
            auto &path = solver->solution->plans[i]->path;
//            if (debug) {
//                std::cout << "agent " << i << ": state " << agents[i].state << std::endl;
//            }
            if (agents[i].state > 0) {
                if (agents[i].state + 1 < path.size()) {
                    path.erase(path.begin(), path.begin() + agents[i].state);
                } else {
                    path.erase(path.begin(), path.begin() + path.size() - 1);
                }
                agents[i].state = 0;
            }
            if (debug) {
                std::cout << "agent " << i << "(" << agents[i].current << "->" << agents[i].goal << "): ";
                for (const auto &label: solver->solution->plans[i]->path) {
                    std::cout << "(" << label.state << "," << label.nodeId << ")->";
                }
                std::cout << std::endl;
            }
        }
        return 0;
        */

    std::vector<Agent> savedAgents = agents;

    for (unsigned int i = 0; i < agents.size(); i++) {
//        if (debug) {
//            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current
//                      << ", goal " << agents[i].goal << std::endl;
//        }
        auto &path = solver->solution->plans[i]->path;
        if (agents[i].state + 1 >= path.size()) {
            // already at goal location
            agents[i].start = agents[i].current;
            continue;
        }
        if (agents[i].blocked) {
            // blocked agent starts at current location
            agents[i].start = agents[i].current;
        } else {
            // unblocked agents starts at next location
            auto state = agents[i].state;
            while (++state < path.size()) {
                auto nodeId = path[state].nodeId;
                if (nodeId != agents[i].current) {
                    agents[i].start = nodeId;
                    break;
                }
            }
            if (state >= path.size()) {
                // already at goal location
                std::cerr << "error" << std::endl;
                agents[i].start = agents[i].current;
            }
        }
    }
//        auto newSolver = std::shared_ptr<Solver>(new EECBSSolver(graph, agents, MakeSpanType::MAXIMUM));
//    std::cerr << "init solver" << std::endl;

    solver->init();
    bool success;
    auto savedPlans = solver->solution->plans;
    if (prioritizedReplan) {
//            success = solver->solve();
        success = solver->solveWithPrioritizedReplan(prioritizedOpt);
        partialReplanCount++;
    } else {
        success = solver->solve();
    }
    if (solver->partialExecutionTime > 0) {
        partialReplanCount++;
        partialReplanTime += solver->partialExecutionTime;
    }
    if (solver->executionTime > 0) {
        fullReplanCount++;
        fullReplanTime += solver->executionTime;
    }
//    if (solver->firstAgentArrivingExecutionTime > 0) {
//        replanCount += currentReplanCount;
//        fullReplanCount += currentFullReplanCount;
//        if (firstAgentArrivingTimestep == 0) {
//            replanBeforeArrivingCount += currentReplanCount;
//            fullReplanBeforeArrivingCount += currentFullReplanCount;
//        }
//    }

    if (!success) {
        solver->solution->plans = std::move(savedPlans);
    }

    // prepend a dummy state to the path of each agent
    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].current = savedAgents[i].current;
        agents[i].timestep = savedAgents[i].timestep;
        agents[i].arrivingTimestep = savedAgents[i].arrivingTimestep;

        if (success) {
            agents[i].state = 0;
            auto &path = solver->solution->plans[i]->path;
            for (auto &label: path) {
                label.state++;
            }
            path.insert(path.begin(), Label{agents[i].current, 0, 0, 0});
        } else {
            agents[i].state = savedAgents[i].state;
            agents[i].start = savedAgents[i].start;
        }
        //        if (debug) {
        //            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current
        //                      << ", goal " << agents[i].goal << std::endl;
        //        }
    }
//    std::cerr << solver->firstAgentArrivingExecutionTime << std::endl;

    if (!success) {
        //        std::cerr << "solve failed" << std::endl;
        //            exit(-1);
        SPDLOG_DEBUG("replan failed in {} seconds", solver->partialExecutionTime + solver->executionTime);
        return -1;
    } else {
        SPDLOG_DEBUG("replan succeeded in {} seconds", solver->partialExecutionTime + solver->executionTime);
        return solver->partialExecutionTime + solver->executionTime;
    }


}
