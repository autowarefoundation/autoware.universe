#ifndef CPPAD_CG_OPERATION_PATH_INCLUDED
#define CPPAD_CG_OPERATION_PATH_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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

#include <cppad/cg/operation_path_node.hpp>
#include <cppad/cg/bidir_graph.hpp>

namespace CppAD {
namespace cg {

/**
 * Finds all paths from a root node to a target node.
 *
 * @param foundGraph stores the graph connecting root to target
 * @param root the current node
 * @param target the node to be found
 * @param bifurcations the current number of bifurcations in the graph
 * @param maxBifurcations the maximum number of bifurcations allowed
 *                        (this function will return if this value was reached)
 */
template<class Base>
inline bool findPathGraph(BidirGraph<Base>& foundGraph,
                          OperationNode<Base>& root,
                          OperationNode<Base>& target,
                          size_t& bifurcations,
                          size_t maxBifurcations = (std::numeric_limits<size_t>::max)()) {
    if (bifurcations >= maxBifurcations) {
        return false;
    }

    if (&root == &target) {
        return true;
    }

    if(foundGraph.contains(root)) {
        return true; // been here and it was saved in foundGraph
    }

    auto* h = root.getCodeHandler();

    if(h->isVisited(root)) {
        return false; // been here but it was not saved in foundGraph
    }

    // not visited yet
    h->markVisited(root); // mark node as visited

    PathNodeEdges<Base>& info = foundGraph[root];

    const auto& args = root.getArguments();

    bool found = false;
    for(size_t i = 0; i < args.size(); ++i) {
        const Argument<Base>& a = args[i];
        if(a.getOperation() != nullptr ) {
            auto& aNode = *a.getOperation();
            if(findPathGraph(foundGraph, aNode, target, bifurcations, maxBifurcations)) {
                foundGraph.connect(info, root, i);
                if(found) {
                    bifurcations++; // multiple ways to get to target
                } else {
                    found = true;
                }
            }
        }
    }

    if(!found) {
        foundGraph.erase(root);
    }

    return found;
}

template<class Base>
inline BidirGraph<Base> CodeHandler<Base>::findPathGraph(OperationNode<Base>& root,
                                                         OperationNode<Base>& target) {
    size_t bifurcations = 0;
    return findPathGraph(root, target, bifurcations);
}

template<class Base>
inline BidirGraph<Base> CodeHandler<Base>::findPathGraph(OperationNode<Base>& root,
                                                         OperationNode<Base>& target,
                                                         size_t& bifurcations,
                                                         size_t maxBifurcations) {
    startNewOperationTreeVisit();

    BidirGraph<Base> foundGraph;

    if (bifurcations <= maxBifurcations) {
        if (&root == &target) {
            foundGraph[root];
        } else {
            CppAD::cg::findPathGraph<Base>(foundGraph, root, target, bifurcations, maxBifurcations);
        }
    }

    return foundGraph;
}


template<class Base>
inline std::vector<std::vector<OperationPathNode<Base> > > CodeHandler<Base>::findPaths(OperationNode<Base>& root,
                                                                                        OperationNode<Base>& code,
                                                                                        size_t max) {
    std::vector<std::vector<OperationPathNode<Base> > > found;

    startNewOperationTreeVisit();

    if (max > 0) {
        std::vector<OperationPathNode<Base> > path2node;
        path2node.reserve(30);
        path2node.push_back(OperationPathNode<Base> (&root, 0));

        if (&root == &code) {
            found.push_back(path2node);
        } else {
            findPaths(path2node, code, found, max);
        }
    }

    return found;
}

template<class Base>
inline void CodeHandler<Base>::findPaths(SourceCodePath& currPath,
                                         OperationNode<Base>& code,
                                         std::vector<SourceCodePath>& found,
                                         size_t max) {

    OperationNode<Base>* currNode = currPath.back().node;
    if (&code == currNode) {
        found.push_back(currPath);
        return;
    }

    const std::vector<Argument<Base> >& args = currNode->getArguments();
    if (args.empty())
        return; // nothing to look in

    if (isVisited(*currNode)) {
        // already searched inside this node
        // any match would have been saved in found
        std::vector<SourceCodePath> pathsFromNode = findPathsFromNode(found, *currNode);
        for (const SourceCodePath& pathFromNode : pathsFromNode) {
            SourceCodePath newPath(currPath.size() + pathFromNode.size());
            std::copy(currPath.begin(), currPath.end(), newPath.begin());
            std::copy(pathFromNode.begin(), pathFromNode.end(), newPath.begin() + currPath.size());
            found.push_back(newPath);
        }

    } else {
        // not visited yet
        markVisited(*currNode); // mark node as visited

        size_t size = args.size();
        for (size_t i = 0; i < size; ++i) {
            OperationNode<Base>* a = args[i].getOperation();
            if (a != nullptr) {
                currPath.push_back(OperationPathNode<Base> (a, i));
                findPaths(currPath, code, found, max);
                currPath.pop_back();
                if (found.size() == max) {
                    return;
                }
            }
        }
    }
}

template<class Base>
inline std::vector<std::vector<OperationPathNode<Base> > > CodeHandler<Base>::findPathsFromNode(const std::vector<SourceCodePath> nodePaths,
                                                                                                OperationNode<Base>& node) {

    std::vector<SourceCodePath> foundPaths;
    std::set<size_t> argsFound;

    for (const SourceCodePath& path : nodePaths) {
        size_t size = path.size();
        for (size_t i = 0; i < size - 1; i++) {
            const OperationPathNode<Base>& pnode = path[i];
            if (pnode.node == &node) {
                if (argsFound.find(path[i + 1].argIndex) == argsFound.end()) {
                    foundPaths.push_back(SourceCodePath(path.begin() + i + 1, path.end()));
                    argsFound.insert(path[i + 1].argIndex);
                }
            }
        }
    }

    return foundPaths;
}

} // END cg namespace
} // END CppAD namespace

#endif