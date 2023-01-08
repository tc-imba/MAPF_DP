//
// Created by liu on 3/1/2022.
//

#ifndef MAPF_DP_SIMULATOR_H
#define MAPF_DP_SIMULATOR_H

#include "../Solver.h"
#include "../EECBSSolver.h"

#include <random>
#include <iostream>
#include <algorithm>

class Simulator {
protected:
    Graph &graph;
    std::vector<Agent> &agents;
    unsigned int seed;

    std::shared_ptr<Solver> solver = nullptr;

    std::set<unsigned int> delayedSet;

public:
    bool debug = false;
    bool replanMode = false;
    std::string delayType;
//    size_t delayInterval = 1;
    double delayRatio = 0.2;

    Simulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed)
            : graph(graph), agents(agents), seed(seed) {}

    void setAgents(const std::vector<Agent> &_agents) {
        agents = _agents;
    }

    void setSolver(std::shared_ptr<Solver> _solver) {
        solver = std::move(_solver);
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

    void updateDelayedSet(unsigned int timestep) {
//        if (delayInterval == 0 || timestep % delayInterval == 0) {
        std::vector<unsigned int> delayed;
        if (delayType == "agent") {
            delayed.resize(agents.size());
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
            }*/
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

    int countCompletedAgents() {
        int count = 0;
        for (unsigned int i = 0; i < agents.size(); i++) {
            if (agents[i].current == agents[i].goal) {
                count++;
            }
        }
        return count;
    }

    virtual int simulate(unsigned int &currentTimestep, unsigned int maxTimeStep,
                         unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) = 0;

    virtual void print(std::ostream &out) const = 0;

    double replan() {
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

        std::vector<unsigned int> savedCurrent(agents.size());

        for (unsigned int i = 0; i < agents.size(); i++) {
            if (debug) {
                std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << std::endl;
            }
            auto &path = solver->solution->plans[i]->path;
            savedCurrent[i] = agents[i].current;
            if (agents[i].state + 1 >= path.size()) {
                // already at goal location
                agents[i].start = agents[i].current;
                continue;
            }
            if (agents[i].blocked) {
                // blocked agent starts at current location
                agents[i].start = agents[i].current;
            } else  {
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
        solver->solve();

        // prepend a dummy state to the path of each agent
        for (unsigned int i = 0; i < agents.size(); i++) {
            agents[i].state = 0;
            agents[i].current = savedCurrent[i];
            if (debug) {
                std::cout << "agent " << i << ": start " << agents[i].start << ", current " << agents[i].current << std::endl;
            }
            auto &path = solver->solution->plans[i]->path;
            for (auto &label : path) {
                label.state++;
            }
            path.insert(path.begin(), Label{agents[i].current, 0, 0, 0});
        }


        return solver->executionTime;
    }

};

#endif //MAPF_DP_SIMULATOR_H
