//
// Created by liuyh on 7/8/2023.
//

#ifndef MAPF_DP_NODEEDGEDEPENDENCYGRAPH_H
#define MAPF_DP_NODEEDGEDEPENDENCYGRAPH_H

#include "DependencyGraph.h"
#include "TopoGraph.h"

class NodeEdgeDependencyGraph : public DependencyGraph {
public:
    std::string removeRedundant = "none";
    std::string snapshotOrder = "none";
    bool useGroup = false;

    enum class SDGNodePairType {
        NodeNode,
        NodeEdge,
        EdgeEdge,
        Fixed,
    };

    bool compareSDGEdgePairWithPartialOrder(const SDGEdgePair &lhs, const SDGEdgePair &rhs) {
        return lhs.first.source.state <= rhs.first.source.state && lhs.first.dest.state >= rhs.first.dest.state && lhs.second.source.state <= rhs.second.source.state && lhs.second.dest.state >= rhs.second.dest.state;
    }

    // each SDGNode has a corresponding SDGData
    struct SDGData {
        // conflicts between the current SDGNode and other SDGNodes
        std::vector<SDGNode> conflicts;
        // determined edges from these SDGNodes (sources) to the current SDGNode (destination)
        // this recording direction is designed for unnecessary blocking
        std::vector<SDGNode> determinedEdgeSources;
        // contain unsettled edges from the current SDGNode (source) to some other SDGNodes (destination)
        std::vector<SDGEdgePair> unsettledEdgePairs;
    };

    typedef std::map<SDGEdgePair, int> SDGEdgePairGroup;

    struct SDGEdgePairData {
        // map<SDGEdgePair, order>
        std::shared_ptr<SDGEdgePairGroup> group = nullptr;
    };

    std::vector<std::vector<SDGData>> sdgData;
    std::set<SDGEdgePair> unsettledEdgePairsSet;
    std::map<SDGEdgePair, SDGEdgePairData> unsettledEdgePairsMap;
    std::set<std::shared_ptr<SDGEdgePairGroup >> unsettledEdgePairGroupsSet;
    std::vector<SharedNodePair> sharedStates;
    std::vector<SDGEdge> savedAddedEdges;

    size_t numAllNodePairs = 0;
    size_t numFixedNodePairs = 0;
    size_t numAddedNodePairs = 0;

    NodeEdgeDependencyGraph(Graph &graph, std::vector<Agent> &agents, std::vector<std::vector<unsigned int>> &paths, const double &firstAgentArrivingTimestep) : DependencyGraph(graph, agents, paths, firstAgentArrivingTimestep){};

    void init();

    SDGNodePair makeSDGNodePair(size_t agentId1, unsigned int state1, size_t agentId2, unsigned int state2);

    SDGEdgePair makeSDGEdgePair(size_t agentId1, unsigned int state1, size_t agentId2, unsigned int state2);

    bool generateUnsettledEdges();

    void initUnsettledEdges();

    void removeRedundantNodePair(std::set<SDGNodePair> &nodePairs, const SDGNodePair &nodePair, int dState1, int dState2);

    void groupNodePair(std::set<SDGNodePair> &nodePairs, const SDGNodePair &nodePair, int dState1, int dState2, int order);

    void generateAddedNodePairsNone(std::set<SDGNodePair> &nnNodePairs, std::set<SDGNodePair> &neNodePairs,
                                    std::set<SDGNodePair> &eeNodePairs, const std::set<SDGNodePair> &fixedNodePairs);


    void generateAddedNodePairsPhysical(std::set<SDGNodePair> &nnNodePairs, std::set<SDGNodePair> &neNodePairs,
                                        std::set<SDGNodePair> &eeNodePairs, const std::set<SDGNodePair> &fixedNodePairs);

    void generateAddedNodePairsGraph(std::set<SDGNodePair> &nnNodePairs, std::set<SDGNodePair> &neNodePairs,
                                     std::set<SDGNodePair> &eeNodePairs, const std::set<SDGNodePair> &fixedNodePairs);

    void groupNodePairs(std::set<SDGNodePair> &nnNodePairs, std::set<SDGNodePair> &neNodePairs,
                        std::set<SDGNodePair> &eeNodePairs, const std::set<SDGNodePair> &fixedNodePairs);

    void initSharedNodes(size_t i, size_t j);

    Graph::NodeEdgeState getNodeEdgeState(size_t agentId, unsigned int state);

    void initSharedStates(size_t agentId1, size_t agentId2);

    std::vector<SDGEdge> updateSharedNode(size_t agentId, unsigned int state, bool dryRun = false);

    std::pair<size_t, size_t> feasibilityCheckHelper(
            std::list<std::shared_ptr<SDGEdgePairGroup>> &unsettledEdgePairGroups,
            bool recursive, bool save
            //            std::vector<std::pair<unsigned int, unsigned int>> &addedEdges
    );

    std::pair<size_t, size_t> feasibilityCheckTest(bool recursive, bool save = false);

    bool singleAgentCheck(size_t agentId);

    void addSavedEdges();

protected:
    void orderEdgesByStart(SDGEdge &edge1, SDGEdge &edge2);

    void orderEdgesByEnd(SDGEdge &edge1, SDGEdge &edge2);

    void orderEdgesByCollision(SDGEdge &edge1, SDGEdge &edge2);
};


#endif//MAPF_DP_NODEEDGEDEPENDENCYGRAPH_H
