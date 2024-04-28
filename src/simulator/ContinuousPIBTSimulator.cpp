//
// Created by liuyh on 5/4/2024.
//

#include "ContinuousPIBTSimulator.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>

unsigned int ContinuousPIBTSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                                               unsigned int delayStart, unsigned int delayInterval) {

    calculateDistances();
    plans.resize(agents.size());
    for (unsigned int i = 0; i < agents.size(); i++) {
        plans[i].push_back(Label{agents[i].start, 0, 0, 0});
    }

    while (currentTimestep < maxTimeStep) {
        SPDLOG_DEBUG("begin timestep {}", currentTimestep);

        // update the location of agents in currentTimestep
        double newCurrentTimestep = currentTimestep;
        for (unsigned int i = 0; i < agents.size(); i++) {
            // agent i arrives at last location in the current plan
            if (agents[i].state < plans[i].back().state && agents[i].arrivingTimestep <= newCurrentTimestep) {
                agents[i].current = plans[i].back().nodeId;
                agents[i].state = plans[i].back().state;
                agents[i].timestep = currentTimestep;
            }
        }

        // sort agents in priority order (lower index = high priority)
        std::vector<unsigned int> agentIds(agents.size());
        std::iota(agentIds.begin(), agentIds.end(), 0);
        std::sort(agentIds.begin(), agentIds.end(), [&](auto agentId1, auto agentId2) {
            bool agent1Arrived = agents[agentId1].current == agents[agentId1].goal;
            bool agent2Arrived = agents[agentId2].current == agents[agentId2].goal;
            if (agent1Arrived && agent2Arrived) {
                return agentId1 < agentId2;
            } else if (agent1Arrived) {
                return false;
            } else if (agent2Arrived) {
                return true;
            } else {
                // TODO: use last moving time as priority
                return agentId1 < agentId2;
            }
        });

        for (auto &agentId: agentIds) {
            PIBT(agentId, agents.size());
        }

        advanceTimestep(currentTimestep);
    }


    return 0;
}

bool ContinuousPIBTSimulator::PIBT(unsigned int agentId1, unsigned int agentId2) {
    auto &ai = agents[agentId1];

    auto &node = graph.getNode(ai.current);
    auto neighborEdges = boost::out_edges(ai.current, graph.g);

    std::vector<unsigned int> C;
    C.emplace_back(ai.current);
    for (auto edge: boost::make_iterator_range(neighborEdges)) { C.emplace_back(edge.m_target); }

    // sort by distance (need cache of distances)
    std::sort(C.begin(), C.end(), [&](auto &nodeId1, auto &nodeId2) {
        return distances[agentId1][nodeId1] < distances[agentId1][nodeId2];
    });

    for (auto nextNodeId: C) { std::cout << agentId1 << " " << nextNodeId << std::endl; }

    return true;
}

void ContinuousPIBTSimulator::calculateDistances() {
    distances.resize(agents.size(), std::vector<double>(graph.getNodeNum()));
    for (unsigned int i = 0; i < agents.size(); i++) {
        boost::dijkstra_shortest_paths(
                graph.g, agents[i].goal,
                boost::weight_map(boost::get(&Graph::Edge::length, graph.g)).distance_map(distances[i].data()));
    }

    std::cout << distances[0][0] << std::endl;
}