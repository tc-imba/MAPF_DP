//
// Created by liuyh on 5/4/2024.
//

#include "ContinuousPIBTSimulator.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/icl/closed_interval.hpp>
#include <boost/icl/continuous_interval.hpp>

#include <Mathematics/DistPointSegment.h>
#include <Mathematics/DistSegmentSegment.h>
#include <Mathematics/IntrOrientedBox2Circle2.h>

unsigned int ContinuousPIBTSimulator::simulate(double &currentTimestep, unsigned int maxTimeStep,
                                               unsigned int delayStart, unsigned int delayInterval) {

    calculateDistances();
    plans.resize(agents.size());
    for (unsigned int i = 0; i < agents.size(); i++) { plans[i].push_back(Label{agents[i].start, 0, 0, 0}); }

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
                agents[i].blocked = true;
            } else {
                agents[i].timestep = currentTimestep;
                agents[i].blocked = false;
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
            if (agents[agentId].blocked) { PIBT(agentId); }
        }

        advanceTimestep(currentTimestep);
    }


    return 0;
}

bool ContinuousPIBTSimulator::PIBT(unsigned int agentId) {
    auto &ai = agents[agentId];

    auto &node = graph.getNode(ai.current);
    auto neighborEdges = boost::out_edges(ai.current, graph.g);

    std::vector<unsigned int> C;
    C.emplace_back(ai.current);
    for (auto edge: boost::make_iterator_range(neighborEdges)) { C.emplace_back(edge.m_target); }

    // sort by distance (need cache of distances)
    std::sort(C.begin(), C.end(),
              [&](auto &nodeId1, auto &nodeId2) { return distances[agentId][nodeId1] < distances[agentId][nodeId2]; });

    for (auto nextNodeId: C) {
        std::cout << agentId << " " << nextNodeId << std::endl;
        double length = 0;
        if (nextNodeId != ai.current) {
            length = graph.getEdge(ai.current, nextNodeId).length;
        }
        auto &nextNode = graph.getNode(nextNodeId);
        auto label = Label{nextNodeId, plans[agentId].back().state + 1, agents[agentId].timestep,
                           agents[agentId].timestep + length};

        // avoid conflict between agent i and the planned path of unblocked agents
        bool conflict = false;
        for (unsigned int k = 0; k < agents.size(); k++) {
            if (k == agentId) continue;
            // if a_k is not blocked, the recursive PIBT is not called, so we should check here
            if (!agents[k].blocked) {
                auto destNodeK = graph.getNode(plans[k].back().nodeId);
                if (plans[k].size() > 1) {
                    // test collision of new path of a_i and the last path of a_k
                    auto srcNodeK = graph.getNode(plans[k][plans[k].size() - 2].nodeId);
                    if (isConflict(label.estimatedTime, node, nextNode, plans[k].back().estimatedTime, srcNodeK,
                                   destNodeK)) {
                        conflict = true;
                        break;
                    }
                    // test collision of the infinite waiting of a_i and the last path of a_k
                    if (isConflict(plans[k].back().estimatedTime, srcNodeK, destNodeK, label.heuristic, nextNode,
                                   nextNode)) {
                        conflict = true;
                        break;
                    }
                }
                // test collision of new path of a_i and the infinite waiting of a_k
                if (isConflict(label.estimatedTime, node, nextNode, plans[k].back().heuristic, destNodeK, destNodeK)) {
                    conflict = true;
                    break;
                }
            }
        }
        if (conflict) continue;

        // set v as the next node of the agent
        if (nextNodeId != ai.current) { plans[agentId].push_back(label); }
        agents[agentId].blocked = false;
        agents[agentId].arrivingTimestep = agents[agentId].timestep + length;

        // avoid conflict between agent i and the infinite waiting of blocked agents
        for (unsigned int k = 0; k < agents.size(); k++) {
            if (k == agentId) continue;
            if (agents[k].blocked) {
                conflict = !PIBT(k);
                if (conflict) break;
            }
        }

        // revert changes if conflict exists
        if (conflict) {
            if (nextNodeId != ai.current) { plans[agentId].pop_back(); }
            agents[agentId].blocked = true;
            agents[agentId].arrivingTimestep = agents[agentId].timestep;
        } else {
            return true;
        }
    }
    return false;
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
bool ContinuousPIBTSimulator::isConflict(double t1, const Graph::Node &srcNode1, const Graph::Node &destNode1,
                                         double t2, const Graph::Node &srcNode2, const Graph::Node &destNode2) {
    //    gte::FIQuery<double, gte::OrientedBox2<double>, gte::Circle2<double>> mQuery;

    gte::Vector2<double> start1 = {(double) srcNode1.x, (double) srcNode1.y};
    gte::Vector2<double> goal1 = {(double) destNode1.x, (double) destNode1.y};
    auto diff1 = goal1 - start1;
    auto distance1 = sqrt(gte::Dot(diff1, diff1));
    gte::Vector2<double> v1 = {diff1[0] / distance1, diff1[1] / distance1};

    gte::Vector2<double> start2 = {(double) srcNode2.x, (double) srcNode2.y};
    gte::Vector2<double> v2;
    double t1end = t1 + distance1, t2end;

    if (srcNode1.index != destNode2.index) {
        // agent 2 moving
        gte::Vector2<double> goal2 = {(double) destNode2.x, (double) destNode2.y};
        auto diff2 = goal2 - start2;
        auto distance2 = sqrt(gte::Dot(diff2, diff2));
        v2 = {diff2[0] / distance2, diff2[1] / distance2};
        t2end = t2 + distance2;
    } else {
        // agent 2 infinite waiting
        v2 = {0, 0};
        t2end = std::numeric_limits<double>::infinity();
    }

    auto interval1 = boost::icl::interval<double>::closed(t1, t1end);
    auto interval2 = boost::icl::interval<double>::closed(t2, t2end);
    if (!boost::icl::intersects(interval1, interval2)) { return false; }
    auto intersection = interval1 & interval2;

    // two circles starts at t=0
    start1 -= v1 * t1;
    start2 -= v2 * t2;
    // change reference axis
    auto start = start1 - start2;
    auto v = v1 - v2;

    auto a = gte::Dot(v, v);
    auto b = 2 * gte::Dot(start, v);
    auto c = gte::Dot(start, start) - 0.5;

    auto delta = b * b - 4 * a * c;
    if (delta < 0) { return false; }
    auto tc1 = (-b - sqrt(delta)) / (2 * a);
    auto tc2 = (-b + sqrt(delta)) / (2 * a);
    auto collisionInterval = boost::icl::interval<double>::closed(tc1, tc2);

    return boost::icl::intersects(intersection, collisionInterval);


    /*if (t1 < t2) {
        start1 += v1 * (t2 - t1);
        t1end -= (t2 - t1);
        t2 -= t1;
        t1 = 0;
    } else if (t2 < t1) {
        start2 += v2 * (t1 - t2);
        t2end -= (t1 - t2);
        t1 -= t2;
        t2 = 0;
    } else {
        t1 = t2 = 0;

    }*/

    /*    gte::Circle2<double> circle(start1, sqrt(2) / 2);
    gte::OrientedBox2<double> box;
    box.extent = {0, 0};
    box.center = start2;

    auto result = mQuery(box, v2, circle, v1);
    if (result.intersectionType != 0 && result.contactTime < distance) {
        return true;
    }*/

    return false;
}
