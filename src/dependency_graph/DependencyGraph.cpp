//
// Created by liuyh on 7/8/2023.
//

#include "DependencyGraph.h"
#include "BoostTopoGraph.h"
#include "ArrayTopoGraph.h"

DependencyGraph::DependencyGraph(Graph &graph, std::vector<Agent> &agents, std::vector<std::vector<unsigned int>> &paths, const double &firstAgentArrivingTimestep) : graph(graph), agents(agents), paths(paths), firstAgentArrivingTimestep(firstAgentArrivingTimestep){};


DependencyGraph::~DependencyGraph() {

}
void DependencyGraph::init() {
    paths.clear();
    paths.resize(agents.size());
    timestamps.clear();
    timestamps.resize(agents.size());
    deadEndStates.clear();
    deadEndStates.resize(agents.size());
    pathTopoNodeIds.clear();
    pathTopoNodeIds.resize(agents.size());

    if (topoGraphType == "boost") {
        std::cout << "boost" << std::endl;
        topoGraph = std::make_unique<BoostTopoGraph>(*this);
    } else if (topoGraphType == "array") {
        std::cout << "array" << std::endl;
        topoGraph = std::make_unique<ArrayTopoGraph>(*this);
    } else {
        exit(-1);
    }
}
