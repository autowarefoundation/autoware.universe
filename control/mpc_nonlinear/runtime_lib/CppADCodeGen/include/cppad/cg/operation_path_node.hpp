#ifndef CPPAD_CG_OPERATION_PATH_NODE_INCLUDED
#define CPPAD_CG_OPERATION_PATH_NODE_INCLUDED
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
struct OperationPathNode {
    /**
     * an operation
     */
    OperationNode<Base>* node;
    /**
     * argument index for the next node in the path
     */
    size_t argIndex;

    inline OperationPathNode() :
            node(nullptr),
            argIndex(0) {
    }

    inline OperationPathNode(OperationNode<Base>* node,
                             size_t argIndex) :
            node(node),
            argIndex(argIndex) {
    }

    inline bool operator<(const OperationPathNode<Base>& right) const {
        return node < right.node || argIndex < right.argIndex;
    }

};

template<class Base>
inline bool operator==(const OperationPathNode<Base>& left,
                       const OperationPathNode<Base>& right) {
    return left.node == right.node && left.argIndex == right.argIndex;
}

template<class Base>
inline bool operator!=(const OperationPathNode<Base>& left,
                       const OperationPathNode<Base>& right) {
    return left.node != right.node || left.argIndex != right.argIndex;
}

} // END cg namespace
} // END CppAD namespace

#endif