#ifndef CPPAD_CG_DUMMY_DERIV_UTIL_INCLUDED
#define CPPAD_CG_DUMMY_DERIV_UTIL_INCLUDED
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

#include <cppad/cg/dae_index_reduction/bipartite_nodes.hpp>

namespace CppAD {
namespace cg {

/**
 * Sorts variable nodes according to the variable differentiation order
 *
 * @param i
 * @param j
 * @return true if i should come before j
 */
template<class Base>
bool sortVnodesByOrder(Vnode<Base>* i,
                       Vnode<Base>* j) {
    return (i->order() > j->order());
}

/**
 * Utility class used to sort variables in the DAE
 */
class DaeVarOrderInfo {
public:
    size_t originalIndex;
    size_t originalIndex0;
    bool hasDerivatives;
    int order;
public:
    inline DaeVarOrderInfo() :
            originalIndex(0),
            originalIndex0(0),
            hasDerivatives(false),
            order(-1) {
    }

    inline DaeVarOrderInfo(size_t moriginalIndex,
                           size_t moriginalIndex0,
                           bool mhasDerivatives,
                           int morder) :
            originalIndex(moriginalIndex),
            originalIndex0(moriginalIndex0),
            hasDerivatives(mhasDerivatives),
            order(morder) {
    }
};

/**
 * Utility class used to sort equations in the DAE system
 */
class DaeEqOrderInfo {
public:
    size_t originalIndex;
    size_t originalIndex0;
    bool differential;
    int assignedVar;
public:
    inline DaeEqOrderInfo() :
            originalIndex(0),
            originalIndex0(0),
            differential(false),
            assignedVar(-1) {
    }

    inline DaeEqOrderInfo(size_t moriginalIndex,
                          size_t moriginalIndex0,
                          bool mdifferential,
                          int massignedVar) :
            originalIndex(moriginalIndex),
            originalIndex0(moriginalIndex0),
            differential(mdifferential),
            assignedVar(massignedVar) {
    }
};

/**
 * Sorts variables based on the differentiation order, whether they are
 * algebraic or differential and the order in the original model
 *
 * @param i
 * @param j
 * @return true if i should come before j
 */
inline bool sortVariablesByOrder(const DaeVarOrderInfo& i,
                                 const DaeVarOrderInfo& j) {
    if (j.order < i.order) {
        return true;
    } else if (j.order > i.order) {
        return false;
    } else if (i.hasDerivatives == j.hasDerivatives) {
        return j.originalIndex > i.originalIndex;
    } else {
        return i.hasDerivatives;
    }
}

/**
 * Sorts equations according to the equation type (differential/algebraic)
 * and original index
 *
 * @param i
 * @param j
 * @return true if i should come before j
 */
inline bool sortEquationByAssignedOrder2(const DaeEqOrderInfo& i,
                                         const DaeEqOrderInfo& j) {
    if (i.differential) {
        if (j.differential)
            return i.assignedVar < j.assignedVar;
        else
            return true;
    } else {
        if (j.differential) {
            return false;
        } else {
            if (i.originalIndex0 == j.originalIndex0) {
                return i.originalIndex == j.originalIndex0;
            } else {
                return i.originalIndex0 < j.originalIndex0;
            }
        }
    }
}

} // END cg namespace
} // END CppAD namespace

#endif