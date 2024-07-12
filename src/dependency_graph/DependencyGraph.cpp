//
// Created by liuyh on 7/8/2023.
//

#include "DependencyGraph.h"

bool DependencyGraph::isEdgeInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
    auto neighborEdges = boost::out_edges(nodeId1, topoGraph);
    return std::any_of(neighborEdges.first, neighborEdges.second,
                       [&](const auto &edge) { return edge.m_target == nodeId2; });
    //    for (auto edge: boost::make_iterator_range(neighborEdges)) {
    //        unsigned int neighborNodeId = edge.m_target;
    //        if (neighborNodeId == nodeId2) {
    //            return true;
    //        }
    //    }
    //    return false;
}

bool DependencyGraph::isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2) {
    auto visitor = TopoGraphBFSVisitor(nodeId2);
    try {
        boost::breadth_first_search(topoGraph, nodeId1, boost::visitor(visitor));
    } catch (...) { return true; }
    return false;
}

bool DependencyGraph::isPathInTopoGraph(const DependencyGraph::SDGEdge &edge) {
    const auto ip = edge.dest.agentId, i = edge.source.agentId;
    const auto jp = edge.dest.state, j = edge.source.state;
    auto ipjp = pathTopoNodeIds[ip][jp];
    return connectedGraph[i][ipjp] <= j;

//    return isPathInTopoGraph(pathTopoNodeIds[ip][jp], pathTopoNodeIds[i][j]);

/*    bool result1 = isPathInTopoGraph(pathTopoNodeIds[ip][jp], pathTopoNodeIds[i][j]);
    bool result2 = connectedGraph[i][ipjp] <= j;
    if (result1 != result2) {
        std::cerr << "error" << std::endl;
        SPDLOG_ERROR("error: {} {} {}, G[{}][{}][{}]={}, j={}", edge, result1, result2, ip, jp, i,
                     connectedGraph[i][ipjp], j);
        exit(-1);
    }
    return result2;*/
}

bool DependencyGraph::addEdgeTopoGraph(const DependencyGraph::SDGEdge &edge) {
    auto [nodeId1, nodeId2] = getTopoEdgeBySDGEdge(edge);
    if (isEdgeInTopoGraph(nodeId1, nodeId2)) { return false; }

    // add an edge from l_{i',j'} (nodeId1) to l_{i,j} (nodeId2)
    const auto ip = edge.source.agentId, i = edge.dest.agentId;
    const auto jp = edge.source.state, j = edge.dest.state;
    boost::add_edge(nodeId1, nodeId2, topoGraph);

    // set G[i'][x][i] = min{G[i'][x][i], j} for all x <= j'
    for (int x = (int) jp; x >= 0; x--) {
        auto ipx = pathTopoNodeIds[ip][x];
        if (connectedGraph[i][ipx] > j) {
            connectedGraph[i][ipx] = j;
//            SPDLOG_INFO("G[{}][{}][{}]={}", ip, x, i, j);
        } else {
            break;
        }
    }

    // set G[i'][x][k] = min{G[i][j][k], G[i'][x][k]} for all k \neq i and x <= j'
    const auto ij = pathTopoNodeIds[i][j];
    for (unsigned int k = 0; k < agents.size(); k++) {
        if (k == i) { continue; }
        for (int x = (int) jp; x >= 0; x--) {
            auto ipx = pathTopoNodeIds[ip][x];
            if (connectedGraph[k][ipx] > connectedGraph[k][ij]) {
                connectedGraph[k][ipx] = connectedGraph[k][ij];
//                SPDLOG_INFO("G[{}][{}][{}]=G[{}][{}][{}]={}", ip, x, k, i, j, k, connectedGraph[k][ij]);
            } else {
                break;
            }
        }
    }

    // for any G[a][b][c] (where a \neq i and i'), if G[a][b][i'] <= j', set G[a][b][c] = min{G[i][j][c], G[a][b][c]}.
    for (unsigned int a = 0; a < agents.size(); a++) {
        if (a == i || a == ip) { continue; }
        for (unsigned int b = 0; b < paths[a].size(); b++) {
            auto ab = pathTopoNodeIds[a][b];
//            SPDLOG_INFO("G[{}][{}][{}]={}, jp={}", a, b, ip, connectedGraph[ip][ab], jp);
            if (connectedGraph[ip][ab] <= jp) {
                for (unsigned int c = 0; c < agents.size(); c++) {
                    auto value = c == i ? j : connectedGraph[c][ij];
                    if (connectedGraph[c][ab] > value) {
                        connectedGraph[c][ab] = value;
//                        SPDLOG_INFO("G[{}][{}][{}]=G[{}][{}][{}]={}", a, b, c, i, j, c, value);
                    }
                }
            } else {
                break;
            }
        }
    }

    return true;
}
