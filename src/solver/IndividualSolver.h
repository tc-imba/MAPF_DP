//
// Created by liu on 16/2/2025.
//

#ifndef INDIVIDUALSOLVER_H
#define INDIVIDUALSOLVER_H

#include <memory>


class IndividualSolver {
public:
    struct Node {
        size_t physicalNodeId;

        int gValue = 0;
        int hValue = 0;
        std::shared_ptr<Node> parent = nullptr;
        double timestep = 0;
        int numOfConflicts = 0;

        bool inOpenList = false;
        bool waitAtGoal = false;
        bool isGoal = false;

        Node() = default;
        Node(const Node&) = default;
        ~Node() = default;

        [[nodiscard]] auto getFValue() const { return gValue + hValue; }

        // used by OPEN (heap) to compare nodes
        // (top of the heap has min f-val, and then highest g-val)
        struct CompareOpen {
            // returns true if lhs > rhs (note -- this gives us *min*-heap).
            bool operator()(const std::shared_ptr<Node> &lhs,
                            const std::shared_ptr<Node> &rhs) const {
                auto lhsFValue = lhs->getFValue();
                auto rhsFValue = rhs->getFValue();

                if (lhsFValue == rhsFValue) {
                    if (lhs->hValue == rhs->hValue) {
                        // break ties randomly in original version
                        // return rand() % 2 == 0;
                        // trying to break ties by physical node id
                        return lhs->physicalNodeId >= rhs->physicalNodeId;
                    }
                    return lhs->hValue >= rhs->hValue;
                }
                // break ties towards smaller h_vals (closer to goal location)
                return lhsFValue >= rhsFValue;
            }
        };

        // used by FOCAL (heap) to compare nodes
        // (top of the heap has min number-of-conflicts)
        struct CompareFocal {
            // returns true if lhs > rhs
            bool operator()(const std::shared_ptr<Node> &lhs,
                            const std::shared_ptr<Node> &rhs) const {
                if (lhs->numOfConflicts == rhs->numOfConflicts) {
                    // break ties towards smaller f_vals (prefer shorter solutions)
                    return CompareOpen()(lhs, rhs);
                }
                // lhs > rhs if it has more conflicts
                return lhs->numOfConflicts >= rhs->numOfConflicts;
            }
        };
    };

    virtual std::string getSolverName() = 0;
};


#endif//INDIVIDUALSOLVER_H
