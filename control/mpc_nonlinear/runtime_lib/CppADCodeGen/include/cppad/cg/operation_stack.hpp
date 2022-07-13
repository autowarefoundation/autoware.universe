#ifndef CPPAD_CG_OPERATION_STACK_HPP
#define CPPAD_CG_OPERATION_STACK_HPP
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
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

/**
 * The steps for a node during the navigation through the graph of operation nodes.
 */
enum class StackNavigationStep {
    Analyze, // analyze the current node (before visiting children)
    ChildrenVisited, // post process node after children have been visited
    Exit // leave node (go to the parent node)
};

/**
 * A simple stack element in the Operation Node stack.
 *
 * @tparam Base
 */
template<class Base>
class SimpleOperationStackData {
private:
    OperationNode<Base>* const _parent;
    const size_t _node;
public:

    inline SimpleOperationStackData(OperationNode<Base>& parent,
                                    size_t nodeIndex) noexcept :
            _parent(&parent),
            _node(nodeIndex) {
    }

    inline OperationNode<Base>& parent() {
        return *_parent;
    }

    inline OperationNode<Base>& node() {
        return *_parent->getArguments()[_node].getOperation();
    }

    inline size_t argumentIndex() const {
        return _node;
    }
};

/**
 * A stack element in the Operation Node stack.
 *
 * @tparam Base
 */
template<class Base>
class OperationStackData :public SimpleOperationStackData<Base> {
public:
    size_t parentNodeScope;
    StackNavigationStep nextStep;

    inline OperationStackData(OperationNode<Base>& parent,
                              size_t nodeIndex,
                              size_t parentNodeScope) noexcept :
            SimpleOperationStackData<Base>(parent, nodeIndex),
            parentNodeScope(parentNodeScope),
            nextStep(StackNavigationStep::Analyze) {
    }

};

/**
 * A Stack of Operation Nodes.
 *
 * @tparam Base
 */
template<class Element>
class BaseOperationStack {
private:
    std::vector<Element> _stack;
public:
    inline BaseOperationStack() {
        _stack.reserve(100);
    }

    inline bool empty() const {
        return _stack.empty();
    }

    inline size_t size() const {
        return _stack.size();
    }

    inline void pop_back() {
        _stack.pop_back();
    }

    inline Element& back() {
        return _stack.back();
    }

    template<class... Args>
    inline void emplace_back(Args&&... args) {
        if (_stack.size() == _stack.capacity()) {
            _stack.reserve((_stack.size() * 3) / 2 + 1);
        }
        _stack.emplace_back(std::forward<Args>(args)...);
    }

    inline Element& operator[](size_t i) {
        return _stack[i];
    }
};

template<class Base>
class OperationStack : public BaseOperationStack<OperationStackData<Base> > {
public:
    inline void pushNodeArguments(OperationNode<Base>& node,
                                  size_t parentNodeScope) {
        auto& args = node.getArguments();

        // append in reverse order so that they are visited in correct forward order
        for (auto itArg = args.rbegin(); itArg != args.rend(); ++itArg) {
            if (itArg->getOperation() != nullptr) {
                size_t index = std::distance(begin(args), itArg.base()) - 1;
                this->emplace_back(node, index, parentNodeScope);
            }
        }
    }
};

template<class Base>
class SimpleOperationStack : public BaseOperationStack<SimpleOperationStackData<Base> > {
public:
    inline void pushNodeArguments(OperationNode<Base>& node) {
        auto& args = node.getArguments();

        // append in reverse order so that they are visited in correct forward order
        for (auto itArg = args.rbegin(); itArg != args.rend(); ++itArg) {
            if (itArg->getOperation() != nullptr) {
                size_t index = std::distance(begin(args), itArg.base()) - 1;
                this->emplace_back(node, index);
            }
        }
    }
};

/**
 * Transverse an operation graph using a depth first algorithm.
 * It allows to execute a function before and after the children of a node have been visited.
 *
 * @tparam Base
 * @tparam FunctionAnalysis
 * @tparam FunctionPostProcess
 *
 * @param root The root operation node.
 * @param currentScopeColor
 * @param nodeAnalysis A function that will be called when the node is visited.
 *                     This function should append to the stack the children which should be visited.
 *                     If there is additional processing after the children have been visited, then it should
 *                     return true (the function nodePostProcessAnalysis will be called).
 * @param nodePostProcessAnalysis A function which may be called (see nodeAnalysis) after all children have been
 *                                visited.
 * @param processRoot Whether or not to include the root in the transversal process
 *                    (call nodeAnalysis/nodePostProcessAnalysis for this node).
 */
template<class Base, typename FunctionAnalysis, typename FunctionPostProcess>
inline void depthFirstGraphNavigation(OperationNode<Base>& root,
                                      size_t currentScopeColor,
                                      FunctionAnalysis& nodeAnalysis,
                                      FunctionPostProcess& nodePostProcessAnalysis,
                                      bool processRoot) {
    OperationStack<Base> stack;

    std::unique_ptr<OperationNode<Base>> fakeSuperRoot;
    if (processRoot) {
        fakeSuperRoot = OperationNode<Base>::makeTemporaryNode(CGOpCode::Alias, {}, {root});
        stack.emplace_back(*fakeSuperRoot, 0, currentScopeColor);
    } else {
        stack.pushNodeArguments(root, currentScopeColor);
    }

    while (!stack.empty()) {

        if (stack.back().nextStep == StackNavigationStep::Analyze) {
            size_t i = stack.size() - 1; // do not use a reference because the stack may be resized

            bool complete = nodeAnalysis(stack[i], stack);

            if (complete)
                stack[i].nextStep = StackNavigationStep::ChildrenVisited;
            else
                stack[i].nextStep = StackNavigationStep::Exit;

        } else if (stack.back().nextStep == StackNavigationStep::ChildrenVisited) {
            nodePostProcessAnalysis(stack.back());
            stack.back().nextStep = StackNavigationStep::Exit;
            stack.pop_back();

        } else {
            stack.pop_back();
        }

    }
}

/**
 * Transverse an operation graph using a depth first algorithm.
 * It allows to execute a function before the children of a node have been visited.
 *
 * @tparam Base
 * @tparam FunctionAnalysis
 *
 * @param root The root operation node (it will NOT be visited by nodeAnalysis, only its children).
 * @param nodeAnalysis A function that will be called when the node is visited.
 *                     This function must append to the stack the children to be visited.
 * @param processRoot Whether or not to include the root in the transversal process
 *                    (call nodeAnalysis/nodePostProcessAnalysis for this node).
 */
template<class Base, typename FunctionAnalysis>
inline void depthFirstGraphNavigation(OperationNode<Base>& root,
                                      FunctionAnalysis& nodeAnalysis,
                                      bool processRoot) {
    SimpleOperationStack<Base> stack;

    std::unique_ptr<OperationNode<Base>> fakeSuperRoot;
    if (processRoot) {
        fakeSuperRoot = OperationNode<Base>::makeTemporaryNode(CGOpCode::Alias, {}, {root});
        stack.emplace_back(*fakeSuperRoot, 0);
    } else {
        stack.pushNodeArguments(root);
    }

    while (!stack.empty()) {
        auto nodeEl = stack.back(); // copy
        stack.pop_back();

        nodeAnalysis(nodeEl, stack);
    }
}

} // END cg namespace
} // END CppAD namespace

#endif
