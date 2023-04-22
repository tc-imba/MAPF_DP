//
// Created by liuyh on 9/4/2023.
//

#include "Simulator.h"

size_t Simulator::combineRandomSeed(
        unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int _seed) {
    size_t result = 0;
    boost::hash_combine(result, nodeId1 ^ nodeId2);
    boost::hash_combine(result, timestep);
    boost::hash_combine(result, _seed);
    return result;
}

double Simulator::averageMakeSpan(MakeSpanType makeSpanType) {
    double result = 0;
    for (const auto &agent: agents) {
        if (makeSpanType == MakeSpanType::MAXIMUM) {
            result = std::max(result, (double) agent.timestep);
        } else if (makeSpanType == MakeSpanType::AVERAGE) {
            result += agent.timestep / (double) agents.size();
        }
//            std::cerr << agent.timestep << " ";
    }
//        std::cerr << std::endl;
    return result;
}

void Simulator::initDelayedIntervals() {
    delayedIntervals.clear();
    delayedShuffleVec.clear();
    if (delayType == "agent") {
        delayedIntervals.resize(agents.size());
        delayedShuffleVec.resize(agents.size());
    } else if (delayType == "edge") {
        delayedIntervals.resize(graph.getEdgeNum());
        delayedShuffleVec.resize(graph.getEdgeNum());
    }
    std::iota(delayedShuffleVec.begin(), delayedShuffleVec.end(), 0);
    maxDelayTimestep = 0;
}

void Simulator::updateDelayedIntervals(double startTimestep, double endTimestep) {
    unsigned int newSeed = combineRandomSeed(0, 0, (size_t) startTimestep, seed);
    if (debug) {
        std::cout << "update delayed interval [" << startTimestep << "," << endTimestep << ") with seed " << newSeed
                  << std::endl;
    }
    std::mt19937 generator(newSeed);
    std::iota(delayedShuffleVec.begin(), delayedShuffleVec.end(), 0);
    std::shuffle(delayedShuffleVec.begin(), delayedShuffleVec.end(), generator);
    for (unsigned int i = 0; i < ((double) delayedShuffleVec.size()) * delayRatio; i++) {
        auto interval = boost::icl::interval<double>::right_open(startTimestep, endTimestep);
        delayedIntervals[delayedShuffleVec[i]] += interval;
//        std::cout << delayedShuffleVec[i] << " " << delayedIntervals[delayedShuffleVec[i]] << std::endl;
    }
}

void Simulator::ensureDelayedIntervals(double targetTimestep, double delayLength) {
    while (maxDelayTimestep < targetTimestep) {
        updateDelayedIntervals(maxDelayTimestep, maxDelayTimestep + delayLength);
        maxDelayTimestep += delayLength;
    }
}

double Simulator::getAgentArrivingTime(double currentTimestep, double delayLength, unsigned int agentId,
                                       const Graph::Edge &edge) {
    double arrivingTimestep = currentTimestep;

    size_t intervalSetIndex = 0;
    if (delayType == "agent") {
        intervalSetIndex = agentId;
    } else if (delayType == "edge") {
        intervalSetIndex = edge.index;
    }
    double remainDistance = edge.length;
    auto &intervalSet = delayedIntervals[intervalSetIndex];
    while (remainDistance > 0) {
        double targetTimestep = arrivingTimestep + remainDistance;
        ensureDelayedIntervals(targetTimestep, delayLength);
        auto moveInterval = boost::icl::interval<double>::right_open(arrivingTimestep,
                                                                     arrivingTimestep + remainDistance);
//        std::cout << intervalSet << std::endl;
        auto collideInterval = intervalSet.find(moveInterval);
        if (collideInterval == intervalSet.end()) {
            // no delay anymore
            arrivingTimestep = targetTimestep;
            break;
        }
//        std::cout << currentTimestep << " " << edge.length << " " << arrivingTimestep << std::endl;
        if (collideInterval->lower() > arrivingTimestep) {
            remainDistance -= collideInterval->lower() - arrivingTimestep;
        }
        arrivingTimestep = collideInterval->upper();
    }
    return arrivingTimestep;
}

/*
void Simulator::updateDelayedSet(unsigned int timestep) {
//        if (delayInterval == 0 || timestep % delayInterval == 0) {
    std::vector<unsigned int> delayed;
    if (delayType == "agent") {
        delayed.resize(agents.size());
        */
/*for (unsigned int i = 0; i < agents.size(); i++) {
            if (checkComplete) {
                auto &state = agents[i].state;
                if (state + 1 < solution->plans[i]->path.size()) {
                    delayed.emplace_back(i);
                }
            } else {
                delayed.emplace_back(i);
            }
            delayed.emplace_back(i);
        }*//*

    } else if (delayType == "edge") {
        delayed.resize(graph.getEdgeNum());
    }
    std::iota(delayed.begin(), delayed.end(), 0);
    unsigned int newSeed = combineRandomSeed(0, 0, timestep, seed);;
    std::mt19937 generator(newSeed);
    std::shuffle(delayed.begin(), delayed.end(), generator);
    delayedSet.clear();
    for (unsigned int i = 0; i < ((double) delayed.size()) * delayRatio; i++) {
        delayedSet.emplace(delayed[i]);
//            std::cout << delayed[i] << " delay" << std::endl;
    }
//        }
}
*/

int Simulator::countCompletedAgents() {
    int count = 0;
    for (unsigned int i = 0; i < agents.size(); i++) {
//        std::cout << i << " " << agents[i].current << " " << agents[i].goal << std::endl;
        if (agents[i].current == agents[i].goal) {
            count++;
        }
    }
    return count;
}

void Simulator::printAgent(size_t i, const std::string &message) {
    auto state = agents[i].state;
    std::cout << "agent " << i << ": ";
    printState(i, state);
    if (message.empty()) {
        std::cout << " -> ";
    } else {
        std::cout << " " << message << " ";
    }
    printState(i, state + 1);
    std::cout << " " << agents[i].arrivingTimestep << std::endl;
}

void Simulator::advanceTimestep(double &currentTimestep) {
    // advance timestep
    // find the first timestep larger than (not equal to) current timestep
    auto it = arrivingTimestepSet.upper_bound(currentTimestep);
    if (it == arrivingTimestepSet.end()) {
        currentTimestep += 1;
        // std::cerr << "warning: no arriving timestep!" << std::endl;
    } else {
        currentTimestep = *it;
        // actually the erase is optional
        arrivingTimestepSet.erase(arrivingTimestepSet.begin(), ++it);
    }
}

double Simulator::replan() {
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
        if (debug) {
            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current
                      << ", goal " << agents[i].goal << std::endl;
        }
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
    solver->init();
    bool success;
    if (prioritizedReplan) {
//            success = solver->solve();
        success = solver->solveWithPrioritizedReplan();
    } else {
        success = solver->solve();
    }
    if (!success) {
        std::cerr << "solve failed" << std::endl;
        return -1;
//            exit(-1);
    }
    // prepend a dummy state to the path of each agent
    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].state = 0;
        agents[i].current = savedAgents[i].current;
        agents[i].timestep = savedAgents[i].timestep;
        agents[i].arrivingTimestep = savedAgents[i].arrivingTimestep;
        if (debug) {
            std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current
                      << ", goal " << agents[i].goal << std::endl;
        }
        auto &path = solver->solution->plans[i]->path;
        for (auto &label: path) {
            label.state++;
        }
        path.insert(path.begin(), Label{agents[i].current, 0, 0, 0});
    }

    return solver->executionTime;
}
