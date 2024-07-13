//
// Created by liuyh on 7/8/2023.
//

#ifndef MAPF_DP_DEPENDENCYGRAPH_H
#define MAPF_DP_DEPENDENCYGRAPH_H

#include "../Graph.h"
#include "../solver/Solver.h"

class TopoGraph;

class DependencyGraph {
public:
    struct SharedNodePair {
        size_t agentId1;
        unsigned int state1;
        size_t agentId2;
        unsigned int state2;
    };

    struct SDGNode {
        size_t agentId;
        unsigned int state;
    };

    friend bool operator<(const SDGNode &lhs, const SDGNode &rhs) {
        if (lhs.agentId == rhs.agentId) return lhs.state < rhs.state;
        return lhs.agentId < rhs.agentId;
    }

    friend bool operator==(const SDGNode &lhs, const SDGNode &rhs) {
        return lhs.agentId == rhs.agentId && lhs.state == rhs.state;
    }

    friend std::ostream &operator<<(std::ostream &out, const SDGNode &node) {
        return out << "(" << node.agentId << "," << node.state << ")";
    }

    struct SDGEdge {
        SDGNode source, dest;
    };

    friend bool operator<(const SDGEdge &lhs, const SDGEdge &rhs) {
        return lhs.source == rhs.source ? lhs.dest < rhs.dest : lhs.source < rhs.source;
    }

    friend bool operator==(const SDGEdge &lhs, const SDGEdge &rhs) {
        return lhs.source == rhs.source && lhs.dest == rhs.dest;
    }

    friend std::ostream &operator<<(std::ostream &out, const SDGEdge &edge) {
        return out << edge.source << "->" << edge.dest;
    }

    typedef std::pair<SDGNode, SDGNode> SDGNodePair;
    typedef std::pair<SDGEdge, SDGEdge> SDGEdgePair;

protected:
    std::vector<std::vector<double>> timestamps;

public:
    Graph &graph;
    std::vector<Agent> &agents;
    std::vector<std::vector<unsigned int>> &paths;
    const double &firstAgentArrivingTimestep;

    bool debug = false;
    bool onlineOpt = false;
    bool groupDetermined = false;
    bool isFastCycleCheck = false;
    std::shared_ptr<Solver> solver = nullptr;
    std::vector<std::vector<unsigned int>> pathTopoNodeIds;
//    Graph::topo_graph_t topoGraph;

    std::string topoGraphType = "array";
    std::unique_ptr<TopoGraph> topoGraph = nullptr;

    //    std::vector<std::vector<unsigned int>> paths;
    std::vector<std::vector<std::pair<size_t, unsigned int>>> deadEndStates;
    std::unordered_map<unsigned int, std::vector<SharedNodePair>> sharedNodes;

    size_t feasibilityCheckUnsettledEdgePairsCount = 0;
    size_t feasibilityCheckTopoCount = 0;
    size_t feasibilityCheckLoopCount = 0;
    size_t feasibilityCheckRecursionCount = 0;

    DependencyGraph(Graph &graph, std::vector<Agent> &agents, std::vector<std::vector<unsigned int>> &paths, const double &firstAgentArrivingTimestep);

    ~DependencyGraph();

    virtual void init();

    std::pair<unsigned int, unsigned int> getTopoEdgeBySDGEdge(const SDGEdge &edge) const {
        return std::make_pair(pathTopoNodeIds[edge.source.agentId][edge.source.state],
                              pathTopoNodeIds[edge.dest.agentId][edge.dest.state]);
    };

//    bool isEdgeInTopoGraph(unsigned int nodeId1, unsigned int nodeId2);
//
//    bool isPathInTopoGraph(unsigned int nodeId1, unsigned int nodeId2);
//
//    bool isPathInTopoGraph(const SDGEdge &edge);
//
//    bool addEdgeTopoGraph(const SDGEdge &edge);

//    bool removeEdgeTopoGraph(const SDGEdge &edge);
};

/*template <> struct fmt::formatter<DependencyGraph::SDGNode>: formatter<string_view> {
    auto format(const DependencyGraph::SDGNode &node, format_context& ctx) const {
        std::string result = "(" + std::to_string(node.agentId) + "," + std::to_string(node.state) + ")";
        return formatter<string_view>::format(result, ctx);
    }
};*/

template <>
struct fmt::formatter<DependencyGraph::SDGNode> : nested_formatter<size_t> {
    auto format(const DependencyGraph::SDGNode &node, format_context& ctx) const {
        return write_padded(ctx, [=](auto out) {
            return format_to(out, "({},{})", nested(node.agentId), nested(node.state));
        });
    }
};

template <>
struct fmt::formatter<DependencyGraph::SDGEdge> : nested_formatter<DependencyGraph::SDGNode> {
    auto format(const DependencyGraph::SDGEdge &edge, format_context& ctx) const {
        return write_padded(ctx, [=](auto out) {
            return format_to(out, "{}->{}", nested(edge.source), nested(edge.dest));
        });
    }
};



#endif//MAPF_DP_DEPENDENCYGRAPH_H
