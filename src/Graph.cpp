//
// Created by liu on 28/4/2021.
//

#include "Graph.h"
#include "CBSSolver.h"
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/graphviz.hpp>

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
//    std::ostream &distOut = !distFileOut.is_open() ? std::cout : distFileOut;

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
    std::cerr << "V: " << V << " E: " << E << " density: " << density << std::endl;
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
Graph::generateGraph(std::vector<std::vector<char>> &gridGraph, const std::string &filename, size_t seed, bool write) {

//    std::ofstream gridFileOut;
//    if (!filename.empty()) gridFileOut.open(filename + ".grid");
//    std::ostream &gridOut = !gridFileOut.is_open() ? std::cout : gridFileOut;
    std::ofstream graphFileOut;
    if (!filename.empty()) graphFileOut.open(filename + ".graph");
    std::ostream &graphOut = !graphFileOut.is_open() ? std::cout : graphFileOut;

    height = gridGraph.size();
    width = gridGraph[0].size();

//    std::mt19937 generator(seed);
//    std::uniform_real_distribution<double> distribution(0, 0.5);
//    std::uniform_real_distribution<double> distribution(minDP, maxDP);

    gridGraphIds.resize(height, std::vector<unsigned int>(width, height * width));

    g.clear();
    unsigned int V = 0;

    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            if (gridGraph[i][j] != '@') {
                gridGraphIds[i][j] = boost::add_vertex(g);
                auto &node = g[gridGraphIds[i][j]];
                node.type = gridGraph[i][j];
                node.x = i;
                node.y = j;
                V++;
            }
        }
    }

    unsigned int E = 0;
    for (unsigned int i = 0; i < height; i++) {
        for (unsigned int j = 0; j < width; j++) {
            if (gridGraph[i][j] != '@') {
                std::vector<std::pair<unsigned int, unsigned int>> neighbors = {{i + 1, j},
                                                                                {i,     j + 1}};
                for (auto [x, y]: neighbors) {
                    if (x < height && y < width && gridGraph[x][y] != '@') {
                        auto p = boost::add_edge(gridGraphIds[i][j], gridGraphIds[x][y], g);
                        auto &edge = g[p.first];
                        edge.length = 1;
                        edge.dp = 0;
                        edge.index = E++;
//                        g[p.first].dp = distribution(generator);
//                        E++;
                    }
                }
            }
        }
    }
    std::cerr << V << " " << E << std::endl;
    nodeNum = V;
    edgeNum = E;

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
            if (gridGraph[i][j] != '@') {
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
            if (gridGraph[i][j] != '@') {
                std::vector<std::pair<unsigned int, unsigned int>> neighbors = {{i + 1, j},
                                                                                {i,     j + 1}};
                for (auto [x, y]: neighbors) {
                    if (x < height && y < width && gridGraph[x][y] != '@') {
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
                                const std::string &filename, size_t seed) {
    assert(obstacles <= height * width);
    std::vector<std::vector<char>> gridGraph(height, std::vector<char>(width, '.'));

    if (loadGridGraph(gridGraph, filename)) {
        std::cerr << "load grid graph from " << filename << ".map" << std::endl;
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
                std::cerr << x << " " << y << " can't be obstacle" << std::endl;
            }
        }
        assert(currentObstacles == obstacles);
        std::cerr << "save grid graph to " << filename << ".map" << std::endl;
        saveGridGraph(gridGraph, filename);
    }
    generateGraph(gridGraph, filename, seed);
}


void Graph::generateWareHouse(unsigned int deliveryWidth, unsigned int maxX, unsigned int maxY,
                              const std::string &filename, size_t seed) {

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

void Graph::generateFileGraph(const std::string &filename) {
    std::vector<std::vector<char>> gridGraph;
    if (loadGridGraph(gridGraph, filename)) {
        std::cerr << "load grid graph from " << filename << ".map" << std::endl;
    } else {
        exit(-1);
    }
    generateGraph(gridGraph, filename, 0, false);
}

double Graph::getHeuristic(unsigned int nodeId1, unsigned int nodeId2) {
    assert(nodeId1 < nodeNum && nodeId2 < nodeNum);
    return distances[nodeId1][nodeId2];
}


const Graph::Edge &Graph::getEdge(unsigned int nodeId1, unsigned int nodeId2) {
    assert(nodeId1 < nodeNum && nodeId2 < nodeNum);
    auto neighborEdges = boost::out_edges(nodeId1, g);
    for (auto edge: boost::make_iterator_range(neighborEdges)) {
        unsigned int neighborNodeId = edge.m_target;
        if (neighborNodeId == nodeId2) {
            return g[edge];
        }
    }
    assert(0);
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
        i++;
        std::cout << "agent " << i << " " << v1[j] << " -> " << v2[j]
                  << " (" << getHeuristic(v1[j], v2[j]) << ")" << std::endl;
    }

    if (agents.size() != i) {
        std::cout << agents.size() << " " << i << std::endl;
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
        std::cerr << "agent " << i << " " << v1[i] << " -> " << v2[i]
                  << " (" << getHeuristic(v1[i], v2[i]) << ")" << std::endl;
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

void Graph::saveAgents(const std::string &mapName, const std::string &filename, const std::vector<Agent> &agents) {
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

