//
// Created by liuyh on 9/4/2023.
//

#include "Simulator.h"
#include <iomanip>

const std::string Simulator::AgentStateLiterals[] = {
        "block", "unblock", "move", "delay", "complete", "fail",
};

size_t Simulator::combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep,
                                    unsigned int _seed) {
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
        //        std::cout << agent.timestep << std::endl;

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
    SPDLOG_DEBUG("update delayed interval [{},{}) with seed {}", startTimestep, endTimestep, newSeed);
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
    double remainDistance = edge.length;
    if (delayLength == 0) { return arrivingTimestep + remainDistance; }
    size_t intervalSetIndex = 0;
    if (delayType == "agent") {
        intervalSetIndex = agentId;
    } else if (delayType == "edge") {
        intervalSetIndex = edge.index;
    }
    auto &intervalSet = delayedIntervals[intervalSetIndex];
    while (remainDistance > 0) {
        double targetTimestep = arrivingTimestep + remainDistance;
        ensureDelayedIntervals(targetTimestep, delayLength);
        auto moveInterval =
                boost::icl::interval<double>::right_open(arrivingTimestep, arrivingTimestep + remainDistance);
        //        std::cout << intervalSet << std::endl;
        auto collideInterval = intervalSet.find(moveInterval);
        if (collideInterval == intervalSet.end()) {
            // no delay anymore
            outputPaths[agentId].push_back(PathNode{arrivingTimestep, agents[agentId].state, AgentState::MOVE});
            arrivingTimestep = targetTimestep;
            break;
        }
        //        std::cout << currentTimestep << " " << edge.length << " " << arrivingTimestep << std::endl;
        if (collideInterval->lower() > arrivingTimestep) {
            outputPaths[agentId].push_back(PathNode{arrivingTimestep, agents[agentId].state, AgentState::MOVE});
            remainDistance -= collideInterval->lower() - arrivingTimestep;
        }
        outputPaths[agentId].push_back(PathNode{collideInterval->lower(), agents[agentId].state, AgentState::DELAY});
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
        if (agents[i].current == agents[i].goal) { count++; }
    }
    return count;
}

void Simulator::printExecutionTime(size_t mapSeed, size_t agentSeed, size_t iteration) {
    timeOutputFile.open(timeOutputFileName, std::ios_base::out | std::ios_base::app);
    if (timeOutputFile.is_open()) {
        timeOutputFile << mapSeed << " " << agentSeed << " " << iteration << " " << executionTimeVec.size()
                       << std::endl;
        timeOutputFile << std::scientific << std::setprecision(12);
        for (unsigned int i = 0; i < executionTimeVec.size(); i++) {
            timeOutputFile << (int) executionTimeVec[i].first << " " << executionTimeVec[i].second << " "
                           << agentCountVec[i].first << " " << agentCountVec[i].second << " " << std::endl;
        }
        timeOutputFile.close();
    }
}

void Simulator::printAgent(size_t i, const std::string &message) {
    std::ostringstream oss;
    auto state = agents[i].state;
    oss << "agent " << i << ": ";
    printState(oss, i, state);
    if (message.empty()) {
        oss << " -> ";
    } else {
        oss << " " << message << " ";
    }
    printState(oss, i, state + 1);
    oss << " " << agents[i].arrivingTimestep;
    SPDLOG_DEBUG("{}", oss.str());
}

void Simulator::saveExecutionTime() {
    bool firstAgentArrived = firstAgentArrivingTimestep > 0;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = now - executionTimeStart;
    executionTimeVec.emplace_back(firstAgentArrived, elapsed_seconds.count());
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
