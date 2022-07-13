#ifndef CPPAD_CG_GRAPH_MOD_INCLUDED
#define CPPAD_CG_GRAPH_MOD_INCLUDED
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

namespace CppAD {
namespace cg {

template<class Base>
inline void CodeHandler<Base>::substituteIndependent(const CG<Base>& indep,
                                                     const CG<Base>& dep,
                                                     bool removeFromIndependents) {
    substituteIndependent(*indep.getOperationNode(), *dep.getOperationNode(), removeFromIndependents);
}

template<class Base>
inline void CodeHandler<Base>::substituteIndependent(OperationNode<Base>& indep,
                                                     OperationNode<Base>& dep,
                                                     bool removeFromIndependents) {
    using std::vector;
    using CGBase = CG<Base>;

    //check if the independent variable belongs to this handler
    size_t indepIndex = getIndependentVariableIndex(indep);

    //check if the dependent variable belongs to this handler
    size_t pos = dep.getHandlerPosition();
    if (pos >= _codeBlocks.size() || &dep != _codeBlocks[pos]) {
        throw CGException("The dependent variable does not belong to this handler");
    }

    // determine the expression for the independent variable
    CGBase dummyExp = solveFor(dep, indep);

    Argument<Base> arg;
    // change the independent variable
    if (dummyExp.isVariable()) {
        arg = Argument<Base> (*dummyExp.getOperationNode());
    } else {
        // create a bogus variable to avoid searching for all occurrences of the independent variable
        arg = Argument<Base> (dummyExp.getValue());
    }

    indep.makeAlias(arg);

    if (removeFromIndependents) {
        // remove the substituted variable from the independent variable vector
        _independentVariables.erase(_independentVariables.begin() + indepIndex);
    }
}

template<class Base>
inline void CodeHandler<Base>::undoSubstituteIndependent(OperationNode<Base>& indep) {
    typename std::vector<OperationNode<Base> *>::const_iterator it =
            std::find(_independentVariables.begin(), _independentVariables.end(), &indep);
    if (it == _independentVariables.end()) {
        throw CGException("Variable not found in the independent variable vector");
    }

    indep.setOperation(CGOpCode::Inv);
}

template<class Base>
inline void CodeHandler<Base>::removeIndependent(OperationNode<Base>& indep) {
    if (indep.getOperationType() != CGOpCode::Alias) {
        throw CGException("Cannot remove independent variable: not an alias");
    }

    typename std::vector<OperationNode<Base> *>::iterator it =
            std::find(_independentVariables.begin(), _independentVariables.end(), &indep);
    if (it == _independentVariables.end()) {
        throw CGException("Variable not found in the independent variable vector");
    }
    _independentVariables.erase(it);
}

} // END cg namespace
} // END CppAD namespace

#endif