//
// Created by liu on 3/1/2022.
//

#ifndef MAPF_DP_SIMULATOR_H
#define MAPF_DP_SIMULATOR_H

#include "../solver/Solver.h"
#include <boost/icl/interval_set.hpp>
//#include "../EECBSSolver.h"

#include <random>
#include <iostream>
#include <algorithm>
#include <fstream>


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
    std::vector<std::pair<bool, double> > executionTimeVec;
    std::set<double> arrivingTimestepSet;

private:
    void updateDelayedIntervals(double start, double length);

public:
    bool debug = false;
    bool replanMode = false;
    bool prioritizedReplan = false;
    std::string delayType;
    std::string outputFileName;
    std::string timeOutputFileName;

    std::ofstream outputFile;
    std::ofstream timeOutputFile;


//    size_t delayInterval = 1;
    double delayRatio = 0.2;
    double firstAgentArrivingTimestep = 0;
    double executionTime = 0;

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

    virtual int simulate(double &currentTimestep, unsigned int maxTimeStep,
                         unsigned int delayStart = INT_MAX, unsigned int delayInterval = INT_MAX) = 0;

    virtual void print(std::ostream &out) const = 0;

    void printExecutionTime(size_t iteration) {
        timeOutputFile.open(timeOutputFileName, std::ios_base::app);
        if (timeOutputFile.is_open()) {
            timeOutputFile << iteration << " " << executionTimeVec.size() << std::endl;
            for (auto &p : executionTimeVec) {
                timeOutputFile << (int) p.first << " " << p.second << std::endl;
            }
            timeOutputFile.close();
        }
    }

    virtual void printState(size_t i, unsigned int state) = 0;

    void printAgent(size_t i, const std::string& message);

    void advanceTimestep(double &currentTimestep);

    double replan();

};

#endif //MAPF_DP_SIMULATOR_H
