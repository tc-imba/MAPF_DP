//
// Created by liuyh on 13/7/2024.
//

#include "ArrayTopoGraph.h"

void ArrayTopoGraph::init() {
    topoGraphNodeNum = 0;
    for (size_t i = 0; i < depGraph.agents.size(); i++) { topoGraphNodeNum += depGraph.paths[i].size(); }
    connectedGraph.resize(depGraph.agents.size(),
                          std::vector<unsigned int>(topoGraphNodeNum, std::numeric_limits<unsigned int>::max() / 2));
}

void ArrayTopoGraph::reset() {
    for (size_t i = 0; i < depGraph.agents.size(); i++) {
        std::fill(connectedGraph[i].begin(), connectedGraph[i].end(), std::numeric_limits<unsigned int>::max() / 2);
    }
}

bool ArrayTopoGraph::addEdge(const DependencyGraph::SDGEdge &edge) {
    // add an edge from l_{i',j'} (nodeId1) to l_{i,j} (nodeId2)
    const auto ip = edge.source.agentId, i = edge.dest.agentId;
    const auto jp = edge.source.state, j = edge.dest.state;
    const auto ipjp = depGraph.pathTopoNodeIds[ip][jp];

    // there is already a path from l_{i',j'} to a state smaller or equal to l_{i,j}
    // so we don't need to update anything here
    if (connectedGraph[i][ipjp] <= j) { return true; }

    // set G[i'][x][i] = min{G[i'][x][i], j} for all x <= j'
    for (int x = (int) jp; x >= 0; x--) {
        auto ipx = depGraph.pathTopoNodeIds[ip][x];
        if (connectedGraph[i][ipx] > j) {
            connectedGraph[i][ipx] = j;
            //            SPDLOG_INFO("G[{}][{}][{}]={}", ip, x, i, j);
        } else {
            break;
        }
    }

    // set G[i'][x][k] = min{G[i][j][k], G[i'][x][k]} for all k \neq i and x <= j'
    const auto ij = depGraph.pathTopoNodeIds[i][j];
    for (unsigned int k = 0; k < depGraph.agents.size(); k++) {
        if (k == i) { continue; }
        for (int x = (int) jp; x >= 0; x--) {
            auto ipx = depGraph.pathTopoNodeIds[ip][x];
            if (connectedGraph[k][ipx] > connectedGraph[k][ij]) {
                connectedGraph[k][ipx] = connectedGraph[k][ij];
                //                SPDLOG_INFO("G[{}][{}][{}]=G[{}][{}][{}]={}", ip, x, k, i, j, k, connectedGraph[k][ij]);
            } else {
                break;
            }
        }
    }

    // for any G[a][b][c] (where a \neq i and i'), if G[a][b][i'] <= j', set G[a][b][c] = min{G[i][j][c], G[a][b][c]}.
    for (unsigned int a = 0; a < depGraph.agents.size(); a++) {
        if (a == i || a == ip) { continue; }
        for (unsigned int b = 0; b < depGraph.paths[a].size(); b++) {
            auto ab = depGraph.pathTopoNodeIds[a][b];
            //            SPDLOG_INFO("G[{}][{}][{}]={}, jp={}", a, b, ip, connectedGraph[ip][ab], jp);
            if (connectedGraph[ip][ab] <= jp) {
                for (unsigned int c = 0; c < depGraph.agents.size(); c++) {
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

bool ArrayTopoGraph::removeEdge(const DependencyGraph::SDGEdge &edge) { return false; }

bool ArrayTopoGraph::hasReversedPath(const DependencyGraph::SDGEdge &edge) {
    const auto ip = edge.dest.agentId, i = edge.source.agentId;
    const auto jp = edge.dest.state, j = edge.source.state;
    auto ipjp = depGraph.pathTopoNodeIds[ip][jp];
    return connectedGraph[i][ipjp] <= j;
}

bool ArrayTopoGraph::hasEdge(const DependencyGraph::SDGEdge &edge) { return false; }
