//
// Created by liu on 3/1/2022.
//

#ifndef MAPF_DP_SIMULATOR_H
#define MAPF_DP_SIMULATOR_H

#include "Solver.h"
#include <random>
#include <iostream>
#include <algorithm>

class Simulator {
protected:
    Graph &graph;
    std::vector<Agent> agents;
    unsigned int seed;

    CBSNodePtr solution = nullptr;
    bool debug = false;
    std::set<unsigned int> delayedSet;

public:
    std::string delayType;
//    size_t delayInterval = 1;
    double delayRatio = 0.2;

    Simulator(Graph &graph, const std::vector<Agent> &agents, unsigned int seed)
            : graph(graph), agents(agents), seed(seed) {}

    void setAgents(const std::vector<Agent> &_agents) {
        agents = _agents;
    }

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

};

#endif //MAPF_DP_SIMULATOR_H
