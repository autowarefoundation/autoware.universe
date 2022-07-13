#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_FOR0_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_FOR0_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

/***************************************************************************
 *  Methods related with loop insertion into the operation graph
 **************************************************************************/

template<class Base>
std::vector<CG<Base> > prepareGraphForward0WithLoops(CodeHandler<Base>& handler,
                                                     size_t m, /// range
                                                     const std::vector<CG<Base>>& x, /// independent variables
                                                     LoopFreeModel<Base>* funNoLoops, /// possibly null
                                                     const std::set<LoopModel<Base>*>& loopTapes) {
    using namespace std;
    using namespace loops;

    using CGBase = CG<Base>;

    std::vector<CGBase> y(m);

    // temporaries
    std::vector<CGBase> tmps;

    /**
     * original equations outside the loops 
     */
    if (funNoLoops != nullptr) {
        const std::vector<size_t>& origEq = funNoLoops->getOrigDependentIndexes();

        std::vector<CGBase> depNL = funNoLoops->getTape().Forward(0, x);

        // original equations
        for (size_t e = 0; e < origEq.size(); e++) {
            y[origEq[e]] = depNL[e];
        }

        tmps.resize(depNL.size() - origEq.size());
        for (size_t i = origEq.size(); i < depNL.size(); i++)
            tmps[i - origEq.size()] = depNL[i];
    }

    /**
     * equations in loops
     */
    OperationNode<Base>* iterationIndexDcl = handler.makeIndexDclrNode(LoopModel<Base>::ITERATION_INDEX_NAME);

    for (LoopModel<Base>* itl : loopTapes) {
        LoopModel<Base>& lModel = *itl;
        size_t nIterations = lModel.getIterationCount();
        const std::vector<std::vector<LoopPosition> >& dependents = lModel.getDependentIndexes();

        /**
         * make the loop start
         */
        LoopStartOperationNode<Base>* loopStart = handler.makeLoopStartNode(*iterationIndexDcl, nIterations);

        IndexOperationNode<Base>* iterationIndexOp = handler.makeIndexNode(*loopStart);
        std::set<IndexOperationNode<Base>*> indexesOps;
        indexesOps.insert(iterationIndexOp);

        std::vector<IfElseInfo<Base> > ifElses;

        /**
         * evaluate the loop body
         */
        std::vector<CGBase> indexedIndeps = createIndexedIndependents(handler, lModel, *iterationIndexOp);
        std::vector<CGBase> xl = createLoopIndependentVector(handler, lModel, indexedIndeps, x, tmps);
        if (xl.size() == 0) {
            xl.resize(1); // does not depend on any variable but CppAD requires at least one
            xl[0] = Base(0);
        }
        std::vector<CGBase> yl = lModel.getTape().Forward(0, xl);

        /**
         * make the loop end
         */
        size_t assignOrAdd = 0;

        const std::vector<IndexPattern*>& depPatterns = lModel.getDependentIndexPatterns();
        std::vector<std::pair<CGBase, IndexPattern*> > indexedLoopResults(yl.size());
        for (size_t i = 0; i < yl.size(); i++) {
            std::map<size_t, size_t> locationsIter2Pos;

            for (size_t it = 0; it < nIterations; it++) {
                if (dependents[i][it].original < m) {
                    locationsIter2Pos[it] = dependents[i][it].original;
                }
            }

            indexedLoopResults[i] = createLoopResult(handler, locationsIter2Pos, nIterations,
                                                     yl[i], depPatterns[i], assignOrAdd,
                                                     *iterationIndexOp, ifElses);
        }

        LoopEndOperationNode<Base>* loopEnd = createLoopEnd(handler, *loopStart, indexedLoopResults, indexesOps, assignOrAdd);

        for (size_t i = 0; i < dependents.size(); i++) {
            for (size_t it = 0; it < nIterations; it++) {
                // an additional alias variable is required so that each dependent variable can have its own ID
                size_t e = dependents[i][it].original;
                if (e < m) { // some equations are not present in all iteration
                    y[e] = handler.createCG(*handler.makeNode(CGOpCode::DependentRefRhs,{e}, {*loopEnd}));
                }
            }
        }

        /**
         * move non-indexed expressions outside loop
         */
        moveNonIndexedOutsideLoop(handler, *loopStart, *loopEnd);
    }

    return y;
}

template<class Base>
std::vector<CG<Base> > ModelCSourceGen<Base>::prepareForward0WithLoops(CodeHandler<Base>& handler,
                                                                       const std::vector<CGBase>& x) {
    return prepareGraphForward0WithLoops(handler, _fun.Range(), x, _funNoLoops, _loopTapes);
}

} // END cg namespace
} // END CppAD namespace

#endif
