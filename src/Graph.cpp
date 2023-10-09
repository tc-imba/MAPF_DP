//
// Created by liu on 28/4/2021.
//

#include "Graph.h"
#include "../Continuous-CBS/tinyxml2.h"
#include "utils/math.hpp"

//#include "solver/CBSSolver.h"
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range.hpp>

#include <Mathematics/IntrOrientedBox2Circle2.h>
#include <Mathematics/DistPointSegment.h>
#include <Mathematics/DistSegmentSegment.h>

#include <cmath>
#include <random>
#include <iostream>
#include <iomanip>
#include <fstream>

Graph::Graph() {

}


void Graph::calculateAllPairShortestPath(const std::string &filename, bool dp) {
    // TODO: cache

    std::ofstream distFileOut;
    if (!filename.empty()) distFileOut.open(filename + (dp ? ".dp.distance" : ".distance"));

//    auto vertices = boost::vertices(g);
    auto edges = boost::edges(g);
/*    unsigned int V = vertices.second - vertices.first;
    unsigned int E = 0;
    for (auto it = vertices.first; it != vertices.second; ++it) {
        E += boost::out_degree(*it, g);
    }*/
    unsigned int V = nodeNum, E = edgeNum;

    distances.resize(V, std::vector<double>(V, 0));
//    std::vector<std::vector<double>> distances(V, std::vector<double>(V, 0));


    for (auto it = edges.first; it != edges.second; ++it) {
        if (dp) {
            g[*it]._distance = g[*it].length / (1 - g[*it].dp);
        } else {
            g[*it]._distance = g[*it].length;
        }
    }
    double density = (double) E * 2 / (V) / (V - 1);
    SPDLOG_INFO("calculate all pair shortest path: V={}, E={}, density={}", V, E, density);
    if (density < 0.1) {
        boost::johnson_all_pairs_shortest_paths(g, distances,
                                                boost::weight_map(boost::get(&Edge::_distance, g)));
    } else {
        boost::floyd_warshall_all_pairs_shortest_paths(g, distances,
                                                       boost::weight_map(boost::get(&Edge::_distance, g)));
    }
    distFileOut << V << std::endl;
    for (unsigned int i = 0; i < V; i++) {
        for (unsigned int j = 0; j < V; j++) {
            distFileOut << distances[i][j] << " ";
        }
        distFileOut << std::endl;
    }
}

void Graph::calculateUnweightedAllPairShortestPath() {
    unsigned int V = nodeNum, E = edgeNum;
    distances.resize(V, std::vector<double>(V, 0));
    auto edges = boost::edges(g);
    for (auto it = edges.first; it != edges.second; ++it) {
        g[*it]._distance = g[*it].length;
    }
    double density = (double) E * 2 / (V) / (V - 1);
    if (density < 0.1) {
        boost::johnson_all_pairs_shortest_paths(g, distances,
                                                boost::weight_map(boost::get(&Edge::_distance, g)));
    } else {
        boost::floyd_warshall_all_pairs_shortest_paths(g, distances,
                                                       boost::weight_map(boost::get(&Edge::_distance, g)));
    }
}

bool Graph::isConnected() {
    for (const auto &row: distances) {
        for (const auto &item: row) {
            if (item > (double) distances.size()) {
                return false;
            }
        }
    }
    return true;
}

void Graph::saveGridGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename) {
    std::ofstream gridFileOut;
    if (!filename.empty()) gridFileOut.open(filename + ".map");
    std::ostream &gridOut = !gridFileOut.is_open() ? std::cout : gridFileOut;
    height = gridGraph.size();
    width = gridGraph[0].size();
    gridOut << "type octile" << std::endl;
    gridOut << "height " << height << std::endl;
    gridOut << "width " << width << std::endl;
    gridOut << "map" << std::endl;
    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            gridOut << gridGraph[i][j];
        }
        gridOut << std::endl;
    }
    gridFileOut.close();
}

bool Graph::loadGridGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename) {
    std::ifstream gridFileIn(filename + ".map");
    if (!gridFileIn.is_open()) {
        return false;
    }
    std::string temp;
    gridFileIn >> temp >> temp >> temp >> height >> temp >> width >> temp;
    gridGraph.resize(height, std::vector<char>(width, '.'));
/*    for (int i = 0; i < 4; i++) {
        if (!std::getline(gridFileIn, temp)) {
            return false;
        }
    }
    height = gridGraph.size();
    width = gridGraph[0].size();*/
    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            if (!(gridFileIn >> gridGraph[i][j])) {
                return false;
            }
        }
    }
    return true;
}

void Graph::saveXMLGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename) {
    if (filename.empty()) return;
    auto fullFileName = filename + ".xml";
    tinyxml2::XMLDocument doc;

    doc.InsertEndChild(doc.NewDeclaration());
    auto root = doc.NewElement("root");
    doc.InsertEndChild(root);
    auto map = doc.NewElement("map");
    root->InsertEndChild(map);

    auto width_elem = doc.NewElement("width");
    map->InsertEndChild(width_elem);
    width_elem->SetText((unsigned int) width);
    auto height_elem = doc.NewElement("height");
    map->InsertEndChild(height_elem);
    height_elem->SetText((unsigned int) height);

    auto grid = doc.NewElement("grid");
    map->InsertEndChild(grid);

    for (unsigned int i = 0; i < height; i++) {
        auto row = doc.NewElement("row");
        grid->InsertEndChild(row);
        std::vector<std::string> rowData;
        for (unsigned int j = 0; j < width; j++) {
            rowData.emplace_back(gridGraph[i][j] == '.' ? "0" : "1");
        }
        auto rowText = boost::algorithm::join(rowData, " ");
        row->SetText(rowText.c_str());
    }

    doc.SaveFile(fullFileName.c_str());
}

void Graph::initKNeighbor(unsigned int kNeighbor) {
    this->kNeighbor = kNeighbor;
    std::vector<gte::Vector2<int>> N = {{0, 1},
                                        {1, 0}};

    for (unsigned int n = 2; n < kNeighbor; n++) {
        std::vector<gte::Vector2<int>> temp(N.size() * 2 - 1);
        temp[0] = N[0];
        for (unsigned int j = 1; j < N.size(); j++) {
            temp[2 * j] = N[j];
            temp[2 * j - 1] = N[j - 1] + N[j];
        }
        temp.swap(N);
    }
    neighborDirections.reserve(4 * N.size());
    for (int kx: {-1, 1}) {
        for (int ky: {-1, 1}) {
            for (auto &p: N) {
                neighborDirections.emplace_back(gte::Vector2<int>{p[0] * kx, p[1] * ky});
            }
        }
    }
    std::sort(neighborDirections.begin(), neighborDirections.end());
    neighborDirections.erase(std::unique(neighborDirections.begin(), neighborDirections.end()),
                             neighborDirections.end());
    neighborDirections.shrink_to_fit();
}


std::vector<Graph::Neighbor> Graph::getNeighbors(
        std::vector<std::vector<char>> &gridGraph, unsigned int x, unsigned int y) {
    std::vector<Neighbor> result;
    gte::Vector2<int> start{(int) x, (int) y};
    for (const auto &direction: neighborDirections) {
        auto goal = start + direction;
        if (goal[0] < 0 || goal[0] >= height || goal[1] < 0 || goal[1] >= width) continue;
        auto diff = goal - start;
        double distance = sqrt(gte::Dot(diff, diff));
        if (isPathConflictWithObstacle(gridGraph, start, goal, distance)) continue;
        result.push_back({(unsigned int) goal[0], (unsigned int) goal[1], distance});
    }
    return result;
}

bool Graph::isPathConflictWithObstacle(std::vector<std::vector<char>> &gridGraph,
                                       gte::Vector2<int> start,
                                       gte::Vector2<int> goal,
                                       double distance) {
    auto diff = goal - start;
    gte::Circle2<double> agentCircle({(double) start[0], (double) start[1]}, sqrt(2) / 4);
    gte::Vector2<double> agentVelocity = {diff[0] / distance, diff[1] / distance};

    gte::OrientedBox2<double> obstacleBox;
    obstacleBox.extent = {0.5, 0.5};
    gte::Vector2<double> obstacleVelocity = {0, 0};
    gte::FIQuery<double, gte::OrientedBox2<double>, gte::Circle2<double>> mQuery;

    unsigned int xMin = std::min(start[0], goal[0]);
    unsigned int xMax = std::max(start[0], goal[0]);
    unsigned int yMin = std::min(start[1], goal[1]);
    unsigned int yMax = std::max(start[1], goal[1]);

    for (unsigned int i = xMin; i <= xMax; i++) {
        for (unsigned int j = yMin; j <= yMax; j++) {
            if (gridGraph[i][j] != '@' && gridGraph[i][j] != 'T') continue;
            // @TODO: add optimizations
//            if (i == start.first && j == start.second) return true;
//            if (i == goal.first && j == goal.second) return true;
            obstacleBox.center = {(double) i, (double) j};
            auto result = mQuery(obstacleBox, obstacleVelocity, agentCircle, agentVelocity);
//            std::cout << agentVelocity[0] << " " << agentVelocity[1] << std::endl;
//            std::cout << start.first << " " << start.second << " " << i << " " << j << " " << result.intersectionType << " " << result.contactTime << std::endl;
            if (result.intersectionType != 0 && result.contactTime < distance) {
                return true;
            }
        }
    }
    return false;
}


void Graph::generateDelayProbability(size_t seed, double minDP, double maxDP) {
    std::mt19937 generator(seed);
//    std::uniform_real_distribution<double> distribution(0, 0.5);
    std::uniform_real_distribution<double> distribution(minDP, maxDP);
    auto edges = boost::edges(g);
    for (auto it = edges.first; it != edges.second; ++it) {
        g[*it].dp = distribution(generator);
    }
}


void
Graph::generateGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename, size_t seed,
                     unsigned int kNeighbor, bool write) {

//    std::ofstream gridFileOut;
//    if (!filename.empty()) gridFileOut.open(filename + ".grid");
//    std::ostream &gridOut = !gridFileOut.is_open() ? std::cout : gridFileOut;
    std::ofstream graphFileOut;
    if (!filename.empty()) graphFileOut.open(filename + ".graph");
    std::ostream &graphOut = !graphFileOut.is_open() ? std::cout : graphFileOut;

    height = gridGraph.size();
    width = gridGraph[0].size();
    initKNeighbor(kNeighbor);

//    std::mt19937 generator(seed);
//    std::uniform_real_distribution<double> distribution(0, 0.5);
//    std::uniform_real_distribution<double> distribution(minDP, maxDP);

    gridGraphIds.resize(height, std::vector<unsigned int>(width, height * width));

    g.clear();
    unsigned int V = 0;

    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            if (gridGraph[i][j] != '@' && gridGraph[i][j] != 'T') {
                gridGraphIds[i][j] = boost::add_vertex(g);
                auto &node = g[gridGraphIds[i][j]];
                node.type = gridGraph[i][j];
                node.x = i;
                node.y = j;
                node.index = V++;
            }
        }
    }

    unsigned int E = 0;
    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            if (gridGraph[i][j] != '@' && gridGraph[i][j] != 'T') {
                auto neighbors = getNeighbors(gridGraph, i, j);
                for (auto &neighbor: neighbors) {
//                    std::cout << i << " " << j << " " << neighbor.x << " " << neighbor.y << " " << neighbor.length
//                              << std::endl;
                    // ignore duplicate edges
                    if (getEdgePtr(gridGraphIds[neighbor.x][neighbor.y], gridGraphIds[i][j])) continue;
                    auto p = boost::add_edge(gridGraphIds[i][j], gridGraphIds[neighbor.x][neighbor.y], g);
                    auto &edge = g[p.first];
                    edge.length = neighbor.length;
                    edge.dp = 0;
                    edge.index = E++;
                }
/*                std::vector<std::pair<unsigned int, unsigned int>> neighbors = {{i + 1, j},
                                                                                {i,     j + 1}};
                for (auto [x, y]: neighbors) {
                    if (x < height && y < width && gridGraph[x][y] != '@') {
                        std::cout << i << " " << j << " " << x <<  " " << y << " " << 1 << std::endl;
                        auto p = boost::add_edge(gridGraphIds[i][j], gridGraphIds[x][y], g);
                        auto &edge = g[p.first];
                        edge.length = 1;
                        edge.dp = 0;
                        edge.index = E++;
//                        g[p.first].dp = distribution(generator);
//                        E++;
                    }
                }*/
            }
        }
    }
    SPDLOG_INFO("generate graph: V={}, E={}", V, E);
    nodeNum = V;
    edgeNum = E;
    distances.resize(V, std::vector<double>(V, 0));

    if (write) {
        boost::dynamic_properties dp;
        dp.property("node_id", boost::get(boost::vertex_index, g));
        dp.property("type", boost::get(&Node::type, g));
        dp.property("length", boost::get(&Edge::length, g));
        dp.property("dp", boost::get(&Edge::dp, g));
        boost::write_graphviz_dp(graphOut, g, dp);
    }
}

void Graph::generateUnweightedGraph(std::vector<std::vector<char>> &gridGraph) {
    height = gridGraph.size();
    width = gridGraph[0].size();
    gridGraphIds.resize(height, std::vector<unsigned int>(width, height * width));

    g.clear();
    unsigned int V = 0;

    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            if (gridGraph[i][j] != '@' && gridGraph[i][j] != 'T') {
                gridGraphIds[i][j] = boost::add_vertex(g);
                g[gridGraphIds[i][j]].type = gridGraph[i][j];
                V++;
            } else {
            }
        }
    }

    unsigned int E = 0;
    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            if (gridGraph[i][j] != '@' && gridGraph[i][j] != 'T') {
                std::vector<std::pair<unsigned int, unsigned int>> neighbors = {{i + 1, j},
                                                                                {i,     j + 1}};
                for (auto [x, y]: neighbors) {
                    if (x < height && y < width && gridGraph[i][j] != '@' && gridGraph[i][j] != 'T') {
                        auto p = boost::add_edge(gridGraphIds[i][j], gridGraphIds[x][y], g);
                        g[p.first].length = 1;
                        g[p.first].dp = 0;
                        E++;
                    }
                }
            }
        }
    }
    nodeNum = V;
    edgeNum = E;
}


void Graph::initDelayProbability(double minDP, double maxDP) {
    this->minDP = minDP;
    this->maxDP = maxDP;
}


void Graph::generateRandomGraph(unsigned int height, unsigned int width, unsigned int obstacles,
                                const std::string &filename, size_t seed, unsigned int kNeighbor) {
    graphFilename = filename;
    assert(obstacles <= height * width);
    std::vector<std::vector<char>> gridGraph(height, std::vector<char>(width, '.'));

    if (!noCache && loadGridGraph(gridGraph, filename)) {
        SPDLOG_INFO("load grid graph from {}.map", filename);
    } else {
        std::vector<unsigned int> v(height * width);
        std::iota(v.begin(), v.end(), 0);
        std::mt19937 generator(seed);
        std::shuffle(v.begin(), v.end(), generator);
        unsigned int currentObstacles = 0;
        for (unsigned int i = 0; i < height * width && currentObstacles < obstacles; i++) {
            auto x = v[i] / height, y = v[i] % height;
            gridGraph[x][y] = '@';
            generateUnweightedGraph(gridGraph);
            calculateUnweightedAllPairShortestPath();
            if (isConnected()) {
                currentObstacles++;
            } else {
                gridGraph[x][y] = '.';
                SPDLOG_DEBUG("{} {} can't be obstacle", x, y);
            }
        }
        assert(currentObstacles == obstacles);
        SPDLOG_DEBUG("save grid graph to {}.map", filename);
        saveGridGraph(gridGraph, filename);
    }
    saveXMLGraph(gridGraph, filename);
    generateGraph(gridGraph, filename, seed, kNeighbor);
}


void Graph::generateWareHouse(unsigned int deliveryWidth, unsigned int maxX, unsigned int maxY,
                              const std::string &filename, size_t seed) {
    graphFilename = filename;
    std::vector<std::vector<char>> gridGraph(maxX, std::vector<char>(maxY, '.'));

    for (unsigned int i = 0; i < maxX; i++) {
        for (unsigned int j = 0; j < maxY; j++) {
            if (i % 3 != 0 && j >= 7 && (j - 7) % (deliveryWidth + 1) != deliveryWidth && j < maxY - 7) {
                gridGraph[i][j] = '@';
            }
        }
    }

    /*for (unsigned int i = 1; i < maxX; i += 2) {
        for (unsigned int j = 7; j < maxY - 7; j++) {
            if ((j - 7) % (deliveryWidth + 1) != deliveryWidth) {
                gridGraph[i][j] = 't';
            }
        }
    }*/

    std::vector<unsigned int> parkingY = {0, 1, 2, 3, 4, 5};
    std::vector<unsigned int> taskY = {maxY - 6, maxY - 5, maxY - 4, maxY - 5, maxY - 2, maxY - 1};
    for (unsigned int i = 0; i < maxX; i++) {
        for (unsigned int j: parkingY) {
            gridGraph[i][j] = 'p';
        }
        for (unsigned int j: taskY) {
            gridGraph[i][j] = 't';
        }
    }

    generateGraph(gridGraph, filename, seed);
}

void Graph::generateHardCodedGraph(const std::string &filename, size_t seed) {
    graphFilename = filename;
    unsigned int height = 7, width = 8;

    std::vector<std::vector<char>> gridGraph(height, std::vector<char>(width, '@'));
    for (unsigned int i = 0; i < width; i++) {
        gridGraph[3][i] = '.';
    }
    for (unsigned int i = 0; i < height / 2 + 1; i++) {
        gridGraph[i][4] = '.';
        gridGraph[i][6] = '.';
    }
    for (unsigned int i = height / 2; i < height; i++) {
        gridGraph[i][3] = '.';
    }
//    gridGraph[0][0] = gridGraph[0][width - 1] = 'p';
//    gridGraph[height - 1][0] = gridGraph[height - 1][width - 1] = 't';

    generateGraph(gridGraph, filename, seed);
}

void Graph::generateMAPFBenchmarkGraph(const std::string &filename, unsigned int kNeighbor) {
    graphFilename = filename;
    std::vector<std::vector<char>> gridGraph;
    if (loadGridGraph(gridGraph, filename)) {
        SPDLOG_INFO("load MAPF benchmark graph from {}.map", filename);
    } else {
        exit(-1);
    }
    generateGraph(gridGraph, filename, 0, kNeighbor, false);
    saveXMLGraph(gridGraph, filename);
}

void Graph::generateDOTGraph(const std::string &filename) {
    graphFilename = filename;
    std::ifstream DOTFileIn(filename + ".graph");
    SPDLOG_INFO("load DOT graph from {}.graph", filename);

    boost::dynamic_properties dp;
    dp.property("node_id", boost::get(&Node::index, g));
    dp.property("type", boost::get(&Node::type, g));
    dp.property("length", boost::get(&Edge::length, g));
    dp.property("dp", boost::get(&Edge::dp, g));

    g.clear();
    boost::read_graphviz(DOTFileIn, g, dp);

    auto vertices = boost::vertices(g);
    auto edges = boost::edges(g);

//    for (auto it = vertices.first; it != vertices.second; it++) {
//        std::cerr << *it << " " << g[*it].index << std::endl;
//    }

    for (auto it = edges.first; it != edges.second; it++) {
        g[*it].length = 1;
    }

    nodeNum = std::distance(vertices.first, vertices.second);
    edgeNum = std::distance(edges.first, edges.second);

}

void Graph::generateGraphMLGraph(const std::string &filename) {
    graphFilename = filename;
    std::ifstream graphMLFileIn(filename + ".xml");
    SPDLOG_INFO("load GraphML graph from {}.xml", filename);

    g.clear();

    graphml_graph_t graphml;
    boost::dynamic_properties dp;
    dp.property("coords", boost::get(&GraphMLNode::coords, graphml));
    dp.property("weight", boost::get(&GraphMLEdge::weight, graphml));
    boost::read_graphml(graphMLFileIn, graphml, dp);

    unsigned int V = 0, E = 0;

    auto nodes = boost::vertices(graphml);
    for (auto it = nodes.first; it != nodes.second; ++it) {
        std::vector<std::string> coordinates;
        boost::split(coordinates, graphml[*it].coords, boost::is_any_of(","));
        graphml[*it].x = std::strtod(coordinates[0].c_str(), nullptr);
        graphml[*it].y = std::strtod(coordinates[1].c_str(), nullptr);

        auto v = boost::add_vertex(g);
        auto &node = g[v];
        node.index = V++;
        node.type = '.';
        node.x = graphml[*it].x;
        node.y = graphml[*it].y;
    }

    for (auto it = nodes.first; it != nodes.second; ++it) {
        auto edges = boost::out_edges(g[*it].index, graphml);
        for (auto it2 = edges.first; it2 != edges.second; ++it2) {
            if (getEdgePtr(it2->m_source, it2->m_target)) continue;

            auto &src = graphml[it2->m_source];
            auto &dest = graphml[it2->m_target];
            auto dx = src.x - dest.x;
            auto dy = src.y - dest.y;

            auto p = boost::add_edge(it2->m_source, it2->m_target, g);
            auto &edge = g[p.first];
            edge.length = std::sqrt(dx * dx + dy * dy);
            edge.dp = 0;
            edge.index = E++;
        }
    }

    nodeNum = V;
    edgeNum = E;
    distances.resize(V, std::vector<double>(V, 0));

}


double Graph::getHeuristic(unsigned int nodeId1, unsigned int nodeId2) {
    assert(nodeId1 < nodeNum && nodeId2 < nodeNum);
    return distances[nodeId1][nodeId2];
}

bool Graph::isConflict(const Graph::NodeEdgeState &state1, const Graph::NodeEdgeState &state2) {
    auto getVec = [&](size_t nodeId) {
        auto &node = getNode(nodeId);
        return gte::Vector2<double>{(double) node.x, (double) node.y};
    };
    auto src1 = getVec(state1.srcNodeId);
    auto src2 = getVec(state2.srcNodeId);
    gte::Segment2<double> seg1, seg2;
    double distance;
    if (state1.isNode && state2.isNode) {
        // node-node conflict
        if (!nodeNodeConflict) return false;
        auto diff = src1 - src2;
        distance = sqrt(gte::Dot(diff, diff));
    } else {
        if (!state1.isNode) {
            auto dest1 = getVec(state1.destNodeId);
            seg1 = {src1, dest1};
        }
        if (!state2.isNode) {
            auto dest2 = getVec(state2.destNodeId);
            seg2 = {src2, dest2};
        }
        if (!state1.isNode && !state2.isNode) {
            // edge-edge conflict
            if (!edgeEdgeConflict) return false;
            gte::DCPQuery<double, gte::Segment2<double>, gte::Segment2<double>> mQuery;
            auto result = mQuery(seg1, seg2);
            distance = result.distance;
        } else {
            // node-edge conflict
            if (!nodeEdgeConflict) return false;
            gte::DCPQuery<double, gte::Vector2<double>, gte::Segment2<double>> mQuery;
            if (!state1.isNode) {
                auto result = mQuery(src2, seg1);
                distance = result.distance;
            } else {
                auto result = mQuery(src1, seg2);
                distance = result.distance;
            }
        }
    }
    auto agentRadiusSum = state1.agentRadius + state2.agentRadius;
    if (!is_close_no_less_than(distance, agentRadiusSum, epsilon)) {
        if (debug) {
            std::ostringstream oss;
            oss << "conflict: ";
            oss << "(" << src1[0] << "," << src1[1] << ")";
            if (!state1.isNode) {
                oss << "->(" << seg1.p[1][0] << "," << seg1.p[1][1] << ")";
            }
            oss << " and ";
            oss << "(" << src2[0] << "," << src2[1] << ")";
            if (!state2.isNode) {
                oss << "->(" << seg2.p[1][0] << "," << seg2.p[1][1] << ")";
            }
            oss << " " << distance;
            SPDLOG_DEBUG("{}", oss.str());
            //            if (state1 % 2 == 1 && state2 % 2 == 1) {
            //                auto &node1 = graph.getNode(paths[agentId1][state1 / 2]);
            //                auto &node2 = graph.getNode(paths[agentId1][state1 / 2 + 1]);
            //                auto &node3 = graph.getNode(paths[agentId2][state2 / 2]);
            //                auto &node4 = graph.getNode(paths[agentId2][state2 / 2 + 1]);
            //                SPDLOG_DEBUG("({} {}) -- ({} {}) and ({} {}) -- ({} {})", node1.x, node1.y, node2.x, node2.y, node3.x, node3.y, node4.x, node4.y);
            //            }
        }
        return true;
    }
    return false;
}

const Graph::Node &Graph::getNode(unsigned int nodeId) {
    assert(nodeId < nodeNum);
    return g[nodeId];
}


const Graph::Edge *Graph::getEdgePtr(unsigned int nodeId1, unsigned int nodeId2) {
    assert(nodeId1 < nodeNum && nodeId2 < nodeNum);
    auto neighborEdges = boost::out_edges(nodeId1, g);
    for (auto edge: boost::make_iterator_range(neighborEdges)) {
        unsigned int neighborNodeId = edge.m_target;
        if (neighborNodeId == nodeId2) {
            return &g[edge];
        }
    }
    return nullptr;
}

const Graph::Edge &Graph::getEdge(unsigned int nodeId1, unsigned int nodeId2) {
    auto ptr = getEdgePtr(nodeId1, nodeId2);
    if (ptr) return *ptr;
    SPDLOG_ERROR("can not find edge from {} to {}", nodeId1, nodeId2);
    assert(0);
    exit(-1);
}

std::vector<Agent> Graph::generateRandomAgents(unsigned int agentNum, size_t seed) {
    std::vector<Agent> agents(agentNum);

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

    std::vector<unsigned int> v1(getNodeNum());
    std::vector<unsigned int> v2(getNodeNum());

    std::iota(v1.begin(), v1.end(), 0);
    std::iota(v2.begin(), v2.end(), 0);

    std::shuffle(v1.begin(), v1.end(), generator);
    std::shuffle(v2.begin(), v2.end(), generator);

    unsigned int i, j;

/*    CBSSolver solver(*this, agents);
    solver.init(0, MakeSpanType::AVERAGE);
    auto node = std::make_shared<CBSNode>();
    node->constraint = {INVALID, INVALID, INVALID};
    node->plans.resize(agents.size());
    node->parent = nullptr;
    node->depth = 0;*/

    for (i = 0, j = 0; i < agents.size() && j < getNodeNum(); j++) {
        if (v1[j] == v2[j]) continue;
        auto distance = getHeuristic(v1[j], v2[j]);
        if (distance >= 1000000) continue;
        agents[i].start = v1[j];
        agents[i].goal = v2[j];
/*        agents[i].current = agents[i].start;
        agents[i].timestep = agents[i].waitingTimestep = 0;
        auto plan = solver.focalSearch(*node, i, 0);
        if (!plan->path.empty()) {
            solver.addPathIntoConstraints(plan);
            node->plans[i] = plan;
            i++;
            std::cout << "agent " << i << " " << v1[j] << " -> " << v2[j]
                      << " (" << getHeuristic(v1[j], v2[j]) << ")" << std::endl;
        } else {
            std::cout << "agent " << i << " " << v1[j] << " -> " << v2[j]
                      << " (" << getHeuristic(v1[j], v2[j]) << ") failed" << std::endl;
        }*/
        SPDLOG_DEBUG("agent {}: {} -> {} ({})", i, v1[j], v2[j], getHeuristic(v1[j], v2[j]));
        i++;
    }

    if (agents.size() != i) {
        SPDLOG_ERROR("fail to generate random agents ({}/{})", i, agents.size());
        exit(-1);
    }

    assert(agents.size() == i);
    return agents;
}

std::vector<Agent> Graph::generateWarehouseAgents(unsigned int agentNum, size_t seed, bool swap) {
    std::vector<Agent> agents(agentNum);

    std::mt19937 generator(seed);
    std::vector<unsigned int> v1, v2;

    auto vertices = boost::vertices(g);
    for (auto it = vertices.first; it != vertices.second; ++it) {
        auto type = g[*it].type;
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
        SPDLOG_INFO("agent {}: {} -> {} ({})", i, v1[i], v2[i], getHeuristic(v1[i], v2[i]));
    }
    return agents;

}

std::vector<Agent> Graph::generateHardCodedAgents(unsigned int agentNum) {
    std::vector<Agent> agents(agentNum);

    agents[0].start = gridGraphIds[3][1];
    agents[0].goal = gridGraphIds[3][7];

    agents[1].start = gridGraphIds[0][4];
    agents[1].goal = gridGraphIds[6][3];

    agents[2].start = gridGraphIds[0][6];
    agents[2].goal = gridGraphIds[3][6];

    return agents;
}

std::vector<Agent> Graph::loadXMLAgents(const std::string &filename, unsigned int agentNum, size_t skip) {
    auto fullFileName = filename + ".xml";
    SPDLOG_INFO("load task from {}", fullFileName);
    std::vector<Agent> agents(agentNum);
    tinyxml2::XMLDocument doc;
    doc.LoadFile(fullFileName.c_str());

    auto root = doc.FirstChildElement("root");
    auto agentElem = root->FirstChildElement();
    for (unsigned int i = 0; i < skip; i++) {
        if (!agentElem) {
            SPDLOG_ERROR("too few agents in task file");
            exit(-1);
        }
        agentElem = agentElem->NextSiblingElement();
    }
    for (unsigned int i = 0; i < agents.size(); i++) {
        if (!agentElem) {
            SPDLOG_ERROR("too few agents in task file");
            exit(-1);
        }
        agents[i].start = std::strtoul(agentElem->Attribute("start_id"), nullptr, 10);
        agents[i].goal = std::strtoul(agentElem->Attribute("goal_id"), nullptr, 10);
        SPDLOG_INFO("agent {}: {} -> {} ({})", i, agents[i].start, agents[i].goal, getHeuristic(agents[i].start, agents[i].goal));
        agentElem = agentElem->NextSiblingElement();
    }
    return agents;
}

void Graph::saveScenAgents(const std::string &mapName, const std::string &filename, const std::vector<Agent> &agents) {
    std::ofstream agentFileOut;
    if (!filename.empty()) agentFileOut.open(filename + ".scen");

    agentFileOut << "version 1" << std::endl;

    for (size_t i = 0; i < agents.size(); i++) {
        agentFileOut << i << "\t" << mapName << "\t"
                     << height << "\t" << width << "\t"
                     << g[agents[i].start].y << "\t" << g[agents[i].start].x << "\t"
                     << g[agents[i].goal].y << "\t" << g[agents[i].goal].x << "\t"
                     << distances[agents[i].start][agents[i].goal] << std::endl;
    }

    agentFileOut.close();
}

void Graph::saveXMLAgents(const std::string &filename, const std::vector<Agent> &agents, const std::string &mapType) {
    if (filename.empty()) return;
    auto fullFileName = filename + ".xml";
    tinyxml2::XMLDocument doc;

    doc.InsertEndChild(doc.NewDeclaration());
    auto root = doc.NewElement("root");
    doc.InsertEndChild(root);
    for (size_t i = 0; i < agents.size(); i++) {
        auto agent = root->GetDocument()->NewElement("agent");
        if (mapType == "graphml") {
            agent->SetAttribute("start_id", agents[i].start);
            agent->SetAttribute("goal_id", agents[i].goal);
        } else {
            agent->SetAttribute("start_i", g[agents[i].start].x);
            agent->SetAttribute("start_j", g[agents[i].start].y);
            agent->SetAttribute("goal_i", g[agents[i].goal].x);
            agent->SetAttribute("goal_j", g[agents[i].goal].y);
        }

        root->InsertEndChild(agent);
    }

    doc.SaveFile(fullFileName.c_str());
}

unsigned int Graph::getNodeIdByGridPos(unsigned int x, unsigned int y) {
    if (x >= height || y >= width) {
        return std::numeric_limits<unsigned int>::max();
    }
    return gridGraphIds[x][y];
}

unsigned int Graph::getNodeIdByRealPos(double x, double y) {
    // TODO: optimize this
//    SPDLOG_DEBUG("{} {}", x, y);
    auto vertices = boost::vertices(g);
    for (auto nodeId : boost::make_iterator_range(vertices.first, vertices.second)) {
//        SPDLOG_DEBUG("{} {} {} {}", g[nodeId].x, x, g[nodeId].y, y);
        if (is_close_equal_to(g[nodeId].x, x, 0.0000001) && is_close_equal_to(g[nodeId].y, y, 0.0000001)) {
            return nodeId;
        }
    }
    return std::numeric_limits<unsigned int>::max();
}
