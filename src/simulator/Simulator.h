//
// Created by liu on 3/1/2022.
//

#ifndef MAPF_DP_SIMULATOR_H
#define MAPF_DP_SIMULATOR_H

#include "../solver/Solver.h"
#include <boost/icl/interval_set.hpp>

#include <random>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

class Simulator {
protected:
    Graph &graph;
    std::vector<Agent> &agents;
    unsigned int seed;

    std::shared_ptr<Solver> solver = nullptr;

//    std::set<unsigned int> delayedSet;
    std::vector<unsigned int> delayedShuffleVec;
    std::vector<boost::icl::interval_set<double> > delayedIntervals;
    double maxDelayTimestep = 0;
    std::chrono::steady_clock::time_point executionTimeStart;
    std::vector<std::pair<bool, double> > executionTimeVec;
    std::set<double> arrivingTimestepSet;

    enum class AgentState {
        BLOCK,
        UNBLOCK,
        MOVE,
        DELAY,
        COMPLETE,
        FAIL,
    };

    static const std::string AgentStateLiterals[6];

    struct PathNode {
        double timestep;
        unsigned int state;
        AgentState agentState;
    };

private:
    void updateDelayedIntervals(double start, double length);

public:
    bool debug = false;
    std::string delayType;
    std::string outputFileName;
    std::string timeOutputFileName;

    std::ofstream outputFile;
    std::ofstream timeOutputFile;


//    size_t delayInterval = 1;
    double delayRatio = 0.2;
    double firstAgentArrivingTimestep = 0;
    double executionTime = 0;

    std::vector<std::vector<PathNode>> outputPaths;

    Simulator(Graph &graph, std::vector<Agent> &agents, unsigned int seed)
            : graph(graph), agents(agents), seed(seed) {}

    void setAgents(const std::vector<Agent> &_agents) {
        agents = _agents;
    }

    void setSolver(std::shared_ptr<Solver> _solver) {
        solver = std::move(_solver);
    }

    size_t combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int _seed);

    double averageMakeSpan(MakeSpanType makeSpanType);

    void initDelayedIntervals();

    void ensureDelayedIntervals(double targetTimestep, double delayLength);

    double getAgentArrivingTime(double currentTimestep, double delayLength, unsigned int agentId, const Graph::Edge& edge);

//    void updateDelayedSet(unsigned int timestep);

    int countCompletedAgents();

    void openOutputFiles() {
        if (!outputFileName.empty()) outputFile.open(outputFileName);
//        if (!timeOutputFileName.empty()) timeOutputFile.open(timeOutputFileName);
    }

    void closeOutputFiles() {
        if (outputFile.is_open()) outputFile.close();
//        if (timeOutputFile.is_open()) timeOutputFile.close();
    }

    virtual unsigned int simulate(double &currentTimestep, unsigned int maxTimeStep,
                         unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) = 0;

    virtual void print(std::ostream &out) const = 0;

    virtual void printState(std::ostream &os, size_t i, unsigned int state) = 0;

    void printExecutionTime(size_t mapSeed, size_t agentSeed, size_t iteration);

    void printAgent(size_t i, const std::string& message);

    void saveExecutionTime();

    void advanceTimestep(double &currentTimestep);

};

#endif //MAPF_DP_SIMULATOR_H
