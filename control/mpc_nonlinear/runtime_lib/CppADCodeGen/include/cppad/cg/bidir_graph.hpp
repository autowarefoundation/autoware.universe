#ifndef CPPAD_CG_BIDIR_GRAPH_INCLUDED
#define CPPAD_CG_BIDIR_GRAPH_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

namespace CppAD {
namespace cg {

template<class Base>
class PathNodeEdges {
public:
    using Node = OperationNode<Base>;
    using Path = OperationPathNode<Base>;
public:
    std::vector<size_t> arguments;
    std::vector<Path> usage; // parent node and argument index in that node
};

/**
 * Bidirectional graph used to navigate through the operations graph.
 */
template<class Base>
class BidirGraph {
public:
    using Node = OperationNode<Base>;
    using SourceCodePath = typename CodeHandler<Base>::SourceCodePath;
private:
    std::map<Node*, PathNodeEdges<Base> > graph_;
public:
    inline virtual ~BidirGraph() { }

    inline bool empty() const {
        return graph_.empty();
    }

    inline void connect(Node& node,
                        size_t argument) {
        connect(graph_[&node], node, argument);
    }

    inline void connect(PathNodeEdges<Base>& nodeInfo,
                        Node& node,
                        size_t argument) {
        CPPADCG_ASSERT_UNKNOWN(argument < node.getArguments().size());
        CPPADCG_ASSERT_UNKNOWN(node.getArguments()[argument].getOperation() != nullptr);
        CPPADCG_ASSERT_UNKNOWN(&graph_[&node] == &nodeInfo);

        nodeInfo.arguments.push_back(argument);

        auto* aNode = node.getArguments()[argument].getOperation();
        graph_[aNode].usage.push_back(OperationPathNode<Base>(&node, argument));
    }

    inline bool contains(Node& node) const {
        auto it = graph_.find(&node);
        return it != graph_.end();
    }

    inline PathNodeEdges<Base>* find(Node& node) {
        auto it = graph_.find(&node);
        if (it != graph_.end())
            return &it->second;
        else
            return nullptr;
    }

    inline const PathNodeEdges<Base>* find(Node& node) const {
        auto it = graph_.find(&node);
        if (it != graph_.end())
            return &it->second;
        else
            return nullptr;
    }

    inline bool erase(Node& node) {
        return graph_.erase(&node) > 0;
    }

    inline PathNodeEdges<Base>& operator[](Node& node) {
        return graph_[&node];
    }

    /**
     * Find a path from node to target without any additional bifurcation along
     * the path.
     */
    inline std::vector<SourceCodePath> findSingleBifurcation(Node& expression,
                                                             Node& target,
                                                             size_t& bifIndex) const {

        std::vector<SourceCodePath> paths;
        bifIndex = -1;

        if (empty()) {
            return paths;
        }

        const PathNodeEdges<Base>* tail = find(target);
        if (tail == nullptr)
            return paths;

        paths.reserve(2);
        paths.resize(1);
        paths[0].reserve(20); // path down

        if (tail->usage.empty()) {
            // only one path with one element
            paths[0].push_back(OperationPathNode<Base>(&target, -1));
            return paths;
        }

        paths = findPathUpTo(expression, target);
        if (paths.size() > 1)
            bifIndex = 0;

        if (paths[0][0].node != &expression) {
            /**
             * Add a missing path from the nodes at paths[0][0] to the
             * expression node
             */
            SourceCodePath pathCommon;

            auto* n = paths[0][0].node;
            auto* edges = find(*n);
            CPPADCG_ASSERT_UNKNOWN(edges != nullptr); // must exist

            while (true) {
                n = edges->usage.begin()->node; // ignore other usages for now!!!!

                pathCommon.push_back(*edges->usage.begin());
                if (n == &expression)
                    break;

                edges = find(*n);
                CPPADCG_ASSERT_UNKNOWN(edges != nullptr);
                CPPADCG_ASSERT_UNKNOWN(!edges->usage.empty());
            }

            bifIndex = pathCommon.size();

            std::reverse(pathCommon.begin(), pathCommon.end());
            for (auto& p: paths)
                p.insert(p.begin(), pathCommon.begin(), pathCommon.end());
        }

        return paths;
    }

private:

    /**
     * Find a path from node to target without any additional bifurcation along the path
     */
    std::vector<SourceCodePath> findPathUpTo(Node& node,
                                             Node& target) const {
        auto* n = &node;

        auto* edges = find(*n);
        CPPADCG_ASSERT_UNKNOWN(edges != nullptr); // must exist

        std::vector<SourceCodePath> paths;
        paths.reserve(2);
        paths.resize(1);

        while (!edges->arguments.empty()) {
            if (edges->arguments.size() > 1) {
                // found bifurcation: must restart!
                size_t a1Index = edges->arguments[0];
                const auto& a1 = n->getArguments()[a1Index];
                paths = findPathUpTo(*a1.getOperation(), target);
                if (paths.size() == 2) {
                    return paths;
                }

                size_t a2Index = edges->arguments[1];
                const auto& a2 = n->getArguments()[a2Index];
                auto paths2 = findPathUpTo(*a2.getOperation(), target);
                if (paths2.size() == 2) {
                    return paths2;
                }

                paths[0].insert(paths[0].begin(), OperationPathNode<Base>(n, a1Index));

                paths.resize(2);
                paths[1].reserve(paths2[0].size() + 1);
                paths[1].insert(paths[1].begin(), OperationPathNode<Base>(n, a2Index));
                paths[1].insert(paths[1].begin() + 1, paths2[0].begin(), paths2[0].end());
                return paths;
            }

            size_t argIndex1 = *edges->arguments.begin(); // only one argument
            paths[0].push_back(OperationPathNode<Base>(n, argIndex1));

            n = n->getArguments()[argIndex1].getOperation();
            edges = find(*n);
            CPPADCG_ASSERT_UNKNOWN(edges != nullptr); // must exist
        }

        paths[0].push_back(OperationPathNode<Base>(n, -1));

        return paths;
    }

#if 0
    void findPathDownThenUp() {
        for (const auto& arg0: tail->usage) {
            paths.resize(1);
            paths[0].clear();
            paths[0].push_back(OperationPathNode<Base>(&target, -1));

            Node* n = arg0.node;
            size_t argIndex = arg0.argIndex;

            const PathNodeEdges<Base>* edges = find(*n);
            CPPADCG_ASSERT_UNKNOWN(edges != nullptr);

            while (true) {
                paths[0].push_back(OperationPathNode<Base>(n, argIndex));

                if(edges->arguments.size() != 1)
                    break; // a bifurcation

                if(edges->usage.empty())
                    break;
                n = edges->usage.begin()->node; // ignore other usages for now!!!!
                argIndex = edges->usage.begin()->argIndex;

                edges = find(*n);
                CPPADCG_ASSERT_UNKNOWN(edges != nullptr);
            }

            CPPADCG_ASSERT_UNKNOWN(!edges->arguments.empty());

            //if(edges->arguments.size() > 2) {
            //    continue; // should not use this???
            //}

            // flip paths[0] so that it starts at bifurcation
            std::reverse(paths[0].begin(), paths[0].end());

            if (edges->arguments.size() == 1) {
                // there is only one path (there are no bifurcations)
                return paths;
            }

            // there is another path up to target
            paths.resize(2);
            paths[1].reserve(20); // path up

            /**
             * go up
             */
            // use the other argument to go up
            auto* n1 = paths[0][1].node;
            size_t argIndex1 = n->getArguments()[edges->arguments[0]].getOperation() == n1? edges->arguments[1]: edges->arguments[0];
            paths[1].push_back(OperationPathNode<Base>(n, argIndex1)); // start at the same location (but different argument index)

            n = n->getArguments()[argIndex1].getOperation();

            edges = find(*n);
            CPPADCG_ASSERT_UNKNOWN(edges != nullptr); // must exist

            while (!edges->arguments.empty()) {
                argIndex1 = *edges->arguments.begin(); // ignore other arguments for now!!!!
                paths[1].push_back(OperationPathNode<Base>(n, argIndex1));

                n = n->getArguments()[argIndex1].getOperation();
                edges = find(*n);
                CPPADCG_ASSERT_UNKNOWN(edges != nullptr); // must exist
            }

            paths[1].push_back(OperationPathNode<Base>(n, -1));

            bifIndex = 0;

            break;
        }
    }
#endif

};

} // END cg namespace
} // END CppAD namespace

#endif
