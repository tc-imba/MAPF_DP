//
// Created by liu on 27/4/2021.
//

#include "Solver.h"
#include <random>
#include <iostream>

void Solver::init(MakeSpanType makeSpanType) {
    this->makeSpanType = makeSpanType;
}

Solver::Solver(Graph &graph, unsigned int agentNum) : graph(graph), agents(agentNum) {

}

void Solver::generateRandomAgents(size_t seed) {
    std::mt19937 generator(seed);

/*    std::vector<unsigned int> v1(graph.getNodeNum());
    std::iota(v1.begin(), v1.end(), 0);
    std::shuffle(v1.begin(), v1.end(), generator);
        std::vector<std::pair<bool, bool>> used(graph.getNodeNum(), {false, false});


    for (unsigned int i = 0; i < agents.size(); i++) {
        agents[i].start = v1[i];
        agents[i].goal = v1[i];
        used[v1[i]].first = true;
        std::vector<unsigned int> v2(graph.getNodeNum());
        std::iota(v2.begin(), v2.end(), 0);
        std::shuffle(v2.begin(), v2.end(), generator);
        for (unsigned int j = 0; j < graph.getNodeNum(); j++) {
            if (v1[i] == v2[j]) continue;
            if (used[v2[j]].second) continue;
            double distance = graph.getHeuristic(v1[i], v2[j]);
//            std::cout << distance << std::endl;
            if (distance >= 30 && distance <= 60) {
                agents[i].goal = v2[j];
                used[v2[j]].second = true;
                std::cout << "agent " << i << " " << agents[i].start << " -> " << agents[i].goal
                          << " (" << distance << ")" << std::endl;
                break;
            }
        }
        if (agents[i].start == agents[i].goal) {
            std::cerr << "error!" << std::endl;
            exit(-1);
        }
    }*/

    std::vector<unsigned int> v1(graph.getNodeNum());
    std::vector<unsigned int> v2(graph.getNodeNum());

    std::iota(v1.begin(), v1.end(), 0);
    std::iota(v2.begin(), v2.end(), 0);

    std::shuffle(v1.begin(), v1.end(), generator);
    std::shuffle(v2.begin(), v2.end(), generator);

    unsigned int i, j;

    for (i = 0, j = 0; i < agents.size() && j < graph.getNodeNum(); j++) {
        if (v1[j] == v2[j]) continue;
        agents[i].start = v1[j];
        agents[i].goal = v2[j];
        std::cout << "agent " << i << " " << v1[j] << " -> " << v2[j]
                  << " (" << graph.getHeuristic(v1[j], v2[j]) << ")" << std::endl;
        i++;
    }

    assert(agents.size() == i);
}

void Solver::generateWarehouseAgents(size_t seed, bool swap) {

    std::mt19937 generator(seed);
    std::vector<unsigned int> v1, v2;

    auto vertices = boost::vertices(graph.g);
    for (auto it = vertices.first; it != vertices.second; ++it) {
        auto type = graph.g[*it].type;
        auto nodeId = std::distance(vertices.first, it);
        if (type == 'p') v1.push_back(nodeId);
        else if (type == 't') v2.push_back(nodeId);
    }

    std::shuffle(v1.begin(), v1.end(), generator);
    std::shuffle(v2.begin(), v2.end(), generator);

    for (int i = 0; i < agents.size(); i++) {
        if (swap && i % 2 == 1) {
            agents[i].start = v2[i];
            agents[i].goal = v1[i];
        } else {
            agents[i].start = v1[i];
            agents[i].goal = v2[i];
        }
        std::cerr << "agent " << i << " " << v1[i] << " -> " << v2[i]
                  << " (" << graph.getHeuristic(v1[i], v2[i]) << ")" << std::endl;
    }

}

double Solver::averageMakeSpan() {
    double result = 0;
    for (const auto &agent:agents) {
        if (makeSpanType == MakeSpanType::MAXIMUM) {
            result = std::max(result, (double) agent.timestep);
        } else if (makeSpanType == MakeSpanType::AVERAGE) {
            result += agent.timestep / (double) agents.size();
        }
    }
    return result;
}

size_t Solver::combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int seed) {
    size_t result = 0;
    boost::hash_combine(result, nodeId1 ^ nodeId2);
    boost::hash_combine(result, timestep);
    boost::hash_combine(result, seed);
    return result;
}

