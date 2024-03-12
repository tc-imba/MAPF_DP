//
// Created by liu on 27/4/2021.
//

#ifndef MAPF_DP_SOLVER_H
#define MAPF_DP_SOLVER_H

#include "../Graph.h"


enum class MakeSpanType {
    MAXIMUM,
    AVERAGE,
    UNKNOWN,
};

constexpr static auto INVALID = std::numeric_limits<unsigned int>::max();

struct Constraint {
    unsigned int agentId;
    unsigned int nodeId;
    unsigned int state;

    bool operator==(const Constraint &that) const {
        return agentId == that.agentId && nodeId == that.nodeId && state == that.state;
    }
};

struct Label {
    unsigned int nodeId;    // id of the node
    unsigned int state;     // state of the agent
    double estimatedTime;   // expected timestep from start
    double heuristic;       // expected timestep to goal
};

struct AgentPlan {
    std::deque<Label> path; // planned path is consisted of labels

    AgentPlan &add(unsigned int nodeId) {
        path.push_back(Label{nodeId, (unsigned int) path.size(), 0, 0});
        return *this;
    }
};

struct CBSNode {
    Constraint constraint;
    double key;
    std::vector<std::shared_ptr<AgentPlan >> plans;
    std::shared_ptr<CBSNode> parent;
    unsigned int depth;
};

struct FocalNode {
    Label label;
    unsigned int conflicts;
    std::shared_ptr<FocalNode> parent;
    bool visited = false;
};

struct AStarNode {
    Label label;
    std::shared_ptr<AStarNode> parent;
    bool visited = false;
};

typedef std::shared_ptr<CBSNode> CBSNodePtr;
typedef std::shared_ptr<FocalNode> FocalNodePtr;
typedef std::shared_ptr<AStarNode> AStarNodePtr;

struct LabelComp {
    bool operator()(Label const &a, Label const &b) const {
        return a.estimatedTime + a.heuristic < b.estimatedTime + b.heuristic;
    }
};

struct CBSNodePtrComp {
    bool operator()(CBSNodePtr const &a, CBSNodePtr const &b) const {
        if (a->key == b->key) return a->depth < b->depth;
        return a->key > b->key;
    }
};

struct FocalNodePtrComp {
//        LabelComp comp;
    double key;

    explicit FocalNodePtrComp(double key) : key(key) {}

    bool operator()(FocalNodePtr const &a, FocalNodePtr const &b) const {
        double keyA = a->label.estimatedTime + a->label.heuristic;
        double keyB = b->label.estimatedTime + b->label.heuristic;
        if (keyA <= key && keyB <= key) {
            if (a->conflicts == b->conflicts) return keyA > keyB;
            return a->conflicts > b->conflicts;
        }
        if (keyA > key && keyB > key) {
            if (keyA == keyB) return a->conflicts > b->conflicts;
            return keyA > keyB;
        }
        if (keyA > key) return true;
        if (keyB > key) return false;
        return true;
//            return !comp(a->label, b->label);
    }
};

struct AStarNodePtrComp {
    constexpr static LabelComp comp = LabelComp();

    bool operator()(AStarNodePtr const &a, AStarNodePtr const &b) const {
        return !comp(a->label, b->label);
    }
};

struct LabelHash {
    size_t operator()(const Label &a) const {
        return boost::hash<std::pair<unsigned int, unsigned int>>{}({a.nodeId, a.state});
    }
};

struct LabelEqual {
    bool operator()(const Label &a, const Label &b) const {
        return a.state == b.state && a.nodeId == b.nodeId;
    }
};

/*    struct ConstraintHash {
        size_t operator()(const Constraint &a) const {
            return boost::hash<std::tuple<unsigned int, unsigned int, unsigned int>>{}({a.agentId, a.nodeId, a.state});
        }
    };*/

class Solver {
protected:
    Graph &graph;
    std::vector<Agent> &agents;
    MakeSpanType makeSpanType;

    // prioritized replan
    std::vector<Agent> savedAgents;
    std::vector<std::shared_ptr<AgentPlan >> savedPlans;
    std::vector<size_t> plannedAgentsMap;

public:
    bool success = false;
    bool debug = false;
    bool useDP = false;
    bool allConstraint = false;
    CBSNodePtr solution;
    unsigned int currentTimestep;
    double executionTime = 0;
    double partialExecutionTime = 0;
    double timeout = 0;
    bool prioritizedReplan = false;

    Solver() = delete;

    Solver(Graph &graph, std::vector<Agent> &agents, MakeSpanType makeSpanType);

    virtual ~Solver() = default;

    virtual std::string getSolverName() = 0;

    static size_t
    combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int seed);

    virtual void init();

//    virtual bool step() = 0;

    virtual bool solve() = 0;

    static bool removeFollowingConflict(std::shared_ptr<Solver> solver);

    double approxAverageMakeSpan(CBSNode &cbsNode);

    static bool solveWithCache(std::shared_ptr<Solver> solver, const std::string &directory, unsigned int agentSeed);

    virtual bool solveWithPrioritizedReplan(bool prioritizedOpt);

    bool solveRetry();

    bool validate(unsigned int maxState);

};


#endif //MAPF_DP_SOLVER_H
