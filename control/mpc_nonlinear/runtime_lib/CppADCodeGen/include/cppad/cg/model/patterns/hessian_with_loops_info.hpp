#ifndef CPPAD_CG_HESSIAN_WITH_LOOPS_INFO_INCLUDED
#define CPPAD_CG_HESSIAN_WITH_LOOPS_INFO_INCLUDED
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

namespace loops {

template<class Base>
class HessianWithLoopsEquationGroupInfo {
public:
    std::vector<std::set<size_t> > evalHessSparsity;
    // (tapeJ1, tapeJ2) -> [positions]
    std::map<pairss, std::vector<HessianElement> > indexedIndexedPositions;
    // (tapeJ1, tapeJ2(j2)) -> [positions]
    std::map<pairss, std::vector<HessianElement> > indexedNonIndexedPositions;
    // (tapeJ1, j2) -> [positions]
    std::map<pairss, std::vector<HessianElement> > indexedTempPositions;
    // (tapeJ1(j1), tapeJ2) -> [positions]
    std::map<pairss, std::vector<HessianElement> > nonIndexedIndexedPositions;
    // (j1, tapeJ2) -> [positions]
    std::map<pairss, std::vector<HessianElement> > tempIndexedPositions;
    // (tapeJ1, j2) -> [k]
    std::map<pairss, std::set<size_t> > indexedTempEvals;
    // [(j1, j2)]
    std::set<pairss> nonIndexedNonIndexedEvals;
    // (j2, j1) -> [k]
    std::map<pairss, std::set<size_t> > nonIndexedTempEvals;
    // (j1, j2) -> [k1]
    std::map<pairss, std::set<size_t> > tempNonIndexedEvals;
    // (j1, j2) -> k1 -> [k2]
    std::map<pairss, std::map<size_t, std::set<size_t> > > tempTempEvals;
    /**
     * Hessian
     */
    std::map<size_t, std::map<size_t, CG<Base> > > hess;

    inline HessianWithLoopsEquationGroupInfo() {
    }

    inline HessianWithLoopsEquationGroupInfo(const LoopModel<Base>& loop) :
        evalHessSparsity(loop.getTapeIndependentCount()) {

    }
};

template<class Base>
class HessianWithLoopsInfo {
public:
    LoopModel<Base>* model;
    //
    std::vector<std::set<size_t> > evalJacSparsity;
    //
    std::vector<HessianWithLoopsEquationGroupInfo<Base> > equationGroups;
    //(j1, j2) -> position
    std::map<pairss, size_t> nonIndexedNonIndexedPosition;
    // (j1 ,j2) -> [k1]
    std::map<pairss, std::set<size_t> > nonLoopNonIndexedNonIndexed;

    LoopStartOperationNode<Base>* loopStart;
    LoopEndOperationNode<Base>* loopEnd;
    IndexOperationNode<Base>* iterationIndexOp;
    std::vector<CG<Base> > x; // loop independent variables
    std::vector<CG<Base> > w;
    /**
     * Jacobian
     */
    std::vector<std::map<size_t, CG<Base> > > dyiDzk;
    std::vector<std::set<size_t> > noLoopEvalHessTempsSparsity;
    std::map<size_t, std::map<size_t, CG<Base> > > dzDxx;

    // if-else branches
    std::vector<IfElseInfo<Base> > ifElses;

    inline HessianWithLoopsInfo() :
        model(nullptr),
        loopStart(nullptr),
        loopEnd(nullptr),
        iterationIndexOp(nullptr) {

    }

    inline HessianWithLoopsInfo(LoopModel<Base>& loop) :
        model(&loop),
        evalJacSparsity(loop.getTapeDependentCount()),
        equationGroups(loop.getEquationsGroups().size(), HessianWithLoopsEquationGroupInfo<Base>(loop)),
        loopStart(nullptr),
        loopEnd(nullptr),
        iterationIndexOp(nullptr) {

    }

    /**
     * Evaluates the Jacobian and the Hessian of the loop model
     * 
     * @param individualColoring whether or not there are atomic
     *                           functions in the model
     */
    inline void evalLoopModelJacobianHessian(bool individualColoring) {
        using std::vector;

        ADFun<CG<Base> >& fun = model->getTape();
        const std::vector<IterEquationGroup<Base> >& eqGroups = model->getEquationsGroups();

        vector<vector<CG<Base> > > vw(1);
        vw[0].resize(w.size());

        vector<CG<Base> > y;

        size_t nEqGroups = equationGroups.size();

        for (size_t g = 0; g < nEqGroups; g++) {
            const IterEquationGroup<Base>& group = eqGroups[g];

            vector<std::map<size_t, std::map<size_t, CG<Base> > > > vhess;

            for (size_t i = 0; i < w.size(); i++) {
                vw[0][i] = Base(0);
            }

            for (size_t itI : group.tapeI) {
                vw[0][itI] = w[itI];
            }

            generateLoopForJacHes(fun, x, vw, y,
                                  model->getJacobianSparsity(),
                                  evalJacSparsity,
                                  dyiDzk,
                                  model->getHessianSparsity(),
                                  equationGroups[g].evalHessSparsity,
                                  vhess,
                                  individualColoring);

            //Hessian
            equationGroups[g].hess = vhess[0];
        }
    }

};

} // END loops namespace

} // END cg namespace
} // END CppAD namespace

#endif
