#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_HESS_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_HESS_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
 *    Copyright (C) 2020 Joao Leal
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

class HessianElement {
public:
    size_t location; // location in the compressed hessian vector
    size_t row;
    unsigned short count; // number of times to be added to that location

    inline HessianElement() :
        location((std::numeric_limits<size_t>::max)()),
        row((std::numeric_limits<size_t>::max)()),
        count(0) {
    }

};

template<class Base>
std::pair<CG<Base>, IndexPattern*> createHessianContribution(CodeHandler<Base>& handler,
                                                             const std::vector<HessianElement>& positions,
                                                             const CG<Base>& ddfdxdx,
                                                             IndexOperationNode<Base>& iterationIndexOp,
                                                             std::vector<IfElseInfo<Base> >& ifElses);

} // END loops namespace

/***************************************************************************
 *  Methods related with loop insertion into the operation graph
 **************************************************************************/

template<class Base>
void ModelCSourceGen<Base>::analyseSparseHessianWithLoops(const std::vector<size_t>& lowerHessRows,
                                                          const std::vector<size_t>& lowerHessCols,
                                                          const std::vector<size_t>& lowerHessOrder,
                                                          std::vector<std::set<size_t> >& noLoopEvalJacSparsity,
                                                          std::vector<std::set<size_t> >& noLoopEvalHessSparsity,
                                                          std::vector<std::map<size_t, std::set<size_t> > >& noLoopEvalHessLocations,
                                                          std::map<LoopModel<Base>*, loops::HessianWithLoopsInfo<Base> >& loopHessInfo,
                                                          bool useSymmetry) {
    using namespace std;
    using namespace CppAD::cg::loops;

    size_t nonIndexdedEqSize = _funNoLoops != nullptr ? _funNoLoops->getOrigDependentIndexes().size() : 0;

    /**
     * determine sparsities
     */
    for (LoopModel<Base>* l : _loopTapes) {
        l->evalJacobianSparsity();
        l->evalHessianSparsity();
    }

    if (_funNoLoops != nullptr) {
        _funNoLoops->evalJacobianSparsity();
        _funNoLoops->evalHessianSparsity();
    }

    size_t m = _fun.Range();
    size_t n = _fun.Domain();

    size_t nnz = lowerHessRows.size();

    noLoopEvalJacSparsity.resize(_funNoLoops != nullptr ? m : 0);
    noLoopEvalHessSparsity.resize(_funNoLoops != nullptr ? n : 0);
    noLoopEvalHessLocations.resize(noLoopEvalHessSparsity.size());

    loopHessInfo.clear();
    for (LoopModel<Base>* loop : _loopTapes) {
        HessianWithLoopsInfo<Base>& loopHessInfol = loopHessInfo[loop];
        loopHessInfol = HessianWithLoopsInfo<Base>(*loop);

        // initialize Hessian information structure
        loopHessInfol.noLoopEvalHessTempsSparsity.resize(_funNoLoops != nullptr ? n : 0);
    }

    auto flipIndices = [&](const std::vector<set<size_t> >& groupHess,
                           size_t tape1,
                           size_t tape2) {
        return useSymmetry && tape1 > tape2 && groupHess[tape2].find(tape1) != groupHess[tape2].end();
    };

    auto flipIndices2 = [&](const std::vector<set<size_t> >& groupHess,
                            size_t tape1,
                            size_t tape2) {
        return useSymmetry && groupHess[tape2].find(tape1) != groupHess[tape2].end();
    };

    /**
     * Load locations in the compressed Hessian
     * d      d y_i
     * d x_j2 d x_j1
     */
    for (size_t eh = 0; eh < nnz; eh++) {
        size_t j1 = lowerHessRows[eh];
        size_t j2 = lowerHessCols[eh];
        size_t e = lowerHessOrder[eh];

        if (_funNoLoops != nullptr) {
            // considers only the pattern for the original equations and leaves out the temporaries
            const std::vector<std::set<size_t> >& dydxx = _funNoLoops->getHessianOrigEqsSparsity();
            if (!dydxx.empty()) {
                if (dydxx[j1].find(j2) != dydxx[j1].end()) {
                    /**
                     * Present in the equations outside the loops
                     */
                    noLoopEvalHessSparsity[j1].insert(j2);
                    noLoopEvalHessLocations[j1][j2].insert(e);
                }
            }
        }

        for (LoopModel<Base>* loop : _loopTapes) {
            size_t nIter = loop->getIterationCount();

            const std::vector<IterEquationGroup<Base> >& eqGroups = loop->getEquationsGroups();
            const std::vector<set<size_t> >& loopJac = loop->getJacobianSparsity();
            HessianWithLoopsInfo<Base>& loopInfo = loopHessInfo.at(loop);

            const std::vector<std::vector<LoopPosition> >& indexedIndepIndexes = loop->getIndexedIndepIndexes();
            const std::vector<LoopPosition>& nonIndexedIndepIndexes = loop->getNonIndexedIndepIndexes();
            const std::vector<LoopPosition>& temporaryIndependents = loop->getTemporaryIndependents();

            size_t nIndexed = indexedIndepIndexes.size();
            size_t nNonIndexed = nonIndexedIndepIndexes.size();

            const LoopPosition* posJ1 = loop->getNonIndexedIndepIndexes(j1);
            const LoopPosition* posJ2 = (j1 == j2) ? posJ1 : loop->getNonIndexedIndepIndexes(j2);

            size_t nEqGroups = loopInfo.equationGroups.size();

            for (size_t g = 0; g < nEqGroups; g++) {
                const IterEquationGroup<Base>& group = eqGroups[g];
                const std::vector<set<size_t> >& groupHess = group.getHessianSparsity();

                /**
                 * indexed - indexed
                 * d      d f_i
                 * d x_l2 d x_l1
                 */
                const std::vector<set<pairss> >& iter2tapeJJ = group.getHessianIndexedIndexedTapeIndexes(j1, j2);
                for (size_t iteration = 0; iteration < iter2tapeJJ.size(); iteration++) {
                    const set<pairss>& tapePairs = iter2tapeJJ[iteration];

                    for (const pairss& itPairs : tapePairs) {
                        size_t tape1 = itPairs.first;
                        size_t tape2 = itPairs.second;
                        pairss tape;

                        bool flip = flipIndices(groupHess, tape1, tape2);
                        if (flip) {
                            tape = pairss(tape2, tape1); // work the symmetry
                        } else {
                            tape = itPairs;
                        }

                        std::vector<HessianElement>& positions = loopInfo.equationGroups[g].indexedIndexedPositions[tape];
                        positions.resize(nIter);

                        positions[iteration].location = e;
                        positions[iteration].row = j1;
                        positions[iteration].count++;

                        loopInfo.equationGroups[g].evalHessSparsity[tape.first].insert(tape.second);
                    }
                }


                /**
                 * indexed - non-indexed 
                 * d      d f_i    ->   d      d f_i
                 * d x_j2 d x_l1        d x_l2 d x_j1
                 */
                if (posJ2 != nullptr) {

                    const std::vector<set<size_t> >& iter2tapeJ1OrigJ2 = group.getHessianIndexedNonIndexedTapeIndexes(j1, j2);
                    for (size_t iteration = 0; iteration < iter2tapeJ1OrigJ2.size(); iteration++) {
                        const set<size_t>& tapeJ1s = iter2tapeJ1OrigJ2[iteration];

                        for (size_t tapeJ1 : tapeJ1s) {

                            bool flip = flipIndices2(groupHess, tapeJ1, posJ2->tape);

                            std::vector<HessianElement>* positions;
                            if (flip) {
                                positions = &loopInfo.equationGroups[g].nonIndexedIndexedPositions[pairss(posJ2->tape, tapeJ1)];
                                loopInfo.equationGroups[g].evalHessSparsity[posJ2->tape].insert(tapeJ1);
                            } else {
                                positions = &loopInfo.equationGroups[g].indexedNonIndexedPositions[pairss(tapeJ1, posJ2->tape)];
                                loopInfo.equationGroups[g].evalHessSparsity[tapeJ1].insert(posJ2->tape);
                            }

                            positions->resize(nIter);
                            (*positions)[iteration].location = e;
                            (*positions)[iteration].row = j1;
                            (*positions)[iteration].count++;
                        }
                    }

                }
            }

            /**
             * indexed - constant z
             * d     d f_i    .   d z_k
             * d z_k d x_l1       d x_j2
             */
            if (_funNoLoops != nullptr) {
                map<size_t, set<size_t> > iter2tapeJ1 = loop->getIndexedTapeIndexes(j1);
                for (const auto& itIter : iter2tapeJ1) {
                    size_t iteration = itIter.first;
                    const set<size_t>& tapeJ1s = itIter.second;
                    const set<const IterEquationGroup<Base>*>& groups = loop->getIterationEquationsGroup()[iteration];

                    for (size_t tapeJ1 : tapeJ1s) {
                        for (const IterEquationGroup<Base>* itg : groups) {
                            const IterEquationGroup<Base>& group = *itg;
                            size_t g = group.index;
                            HessianWithLoopsEquationGroupInfo<Base>& groupInfo = loopInfo.equationGroups[g];
                            const std::vector<set<size_t> >& groupHess = group.getHessianSparsity();

                            auto itz = groupHess[tapeJ1].lower_bound(nIndexed + nNonIndexed);

                            pairss pos(tapeJ1, j2);
                            bool used = false;

                            // loop temporary variables
                            for (; itz != groupHess[tapeJ1].end(); ++itz) {
                                size_t tapeJ = *itz;
                                size_t k = temporaryIndependents[tapeJ - nIndexed - nNonIndexed].original;

                                /**
                                 * check if this temporary depends on j2
                                 */
                                const set<size_t>& sparsity = _funNoLoops->getJacobianSparsity()[nonIndexdedEqSize + k];
                                if (sparsity.find(j2) != sparsity.end()) {
                                    noLoopEvalJacSparsity[nonIndexdedEqSize + k].insert(j2); // element required

                                    size_t tapeK = loop->getTempIndepIndexes(k)->tape;

                                    used = true;

                                    set<size_t>& evals = groupInfo.indexedTempEvals[pos];
                                    evals.insert(k);

                                    groupInfo.evalHessSparsity[tapeJ1].insert(tapeK);
                                }
                            }


                            if (used) {
                                std::vector<HessianElement>& positions = groupInfo.indexedTempPositions[pos];
                                positions.resize(nIter);

                                positions[iteration].location = e;
                                positions[iteration].row = j1;
                                positions[iteration].count++;
                            }

                        }

                    }

                }
            }

            /**
             * non-indexed - indexed
             * d      d f_i
             * d x_l2 d x_j1
             */
            if (posJ1 != nullptr) {

                for (size_t g = 0; g < nEqGroups; g++) {
                    const IterEquationGroup<Base>& group = eqGroups[g];

                    const std::vector<set<size_t> >& iter2TapeJ2 = group.getHessianNonIndexedIndexedTapeIndexes(j1, j2);
                    for (size_t iteration = 0; iteration < iter2TapeJ2.size(); iteration++) {
                        const set<size_t>& tapeJ2s = iter2TapeJ2[iteration];

                        for (size_t tapeJ2 : tapeJ2s) {
                            std::vector<HessianElement>& positions = loopInfo.equationGroups[g].nonIndexedIndexedPositions[pairss(posJ1->tape, tapeJ2)];
                            positions.resize(nIter);
                            positions[iteration].location = e;
                            positions[iteration].row = j1;
                            positions[iteration].count++;

                            loopInfo.equationGroups[g].evalHessSparsity[posJ1->tape].insert(tapeJ2);
                        }
                    }
                }
            }

            /**
             * non-indexed - non-indexed
             * d      d f_i
             * d x_j2 d x_j1
             */
            bool jInNonIndexed = false;
            pairss orig(j1, j2);

            if (posJ1 != nullptr && posJ2 != nullptr) {
                for (size_t g = 0; g < nEqGroups; g++) {
                    const IterEquationGroup<Base>& group = eqGroups[g];

                    const set<pairss>& orig1orig2 = group.getHessianNonIndexedNonIndexedIndexes();
                    if (orig1orig2.find(orig) != orig1orig2.end()) {
                        jInNonIndexed = true;

                        loopInfo.equationGroups[g].nonIndexedNonIndexedEvals.insert(orig);
                        loopInfo.equationGroups[g].evalHessSparsity[posJ1->tape].insert(posJ2->tape);
                    }

                }

                if (jInNonIndexed)
                    loopInfo.nonIndexedNonIndexedPosition[orig] = e;
            }

            /**
             * non-indexed - temporaries
             * d     d f_i   .  d z_k
             * d z_k d x_j1     d x_j2
             */
            if (_funNoLoops != nullptr && posJ1 != nullptr) {

                for (size_t g = 0; g < nEqGroups; g++) {
                    const IterEquationGroup<Base>& group = eqGroups[g];

                    const set<size_t>& hessRow = group.getHessianSparsity()[posJ1->tape];
                    auto itz = hessRow.lower_bound(nIndexed + nNonIndexed);

                    // loop temporary variables
                    for (; itz != hessRow.end(); ++itz) {
                        size_t tapeJ = *itz;
                        size_t k = temporaryIndependents[tapeJ - nIndexed - nNonIndexed].original;

                        // Jacobian of g for k must have j2
                        const set<size_t>& gJacRow = _funNoLoops->getJacobianSparsity()[nonIndexdedEqSize + k];
                        if (gJacRow.find(j2) != gJacRow.end()) {
                            noLoopEvalJacSparsity[nonIndexdedEqSize + k].insert(j2); // element required

                            if (!jInNonIndexed) {
                                jInNonIndexed = true;
                                CPPADCG_ASSERT_KNOWN(loopInfo.nonIndexedNonIndexedPosition.find(orig) == loopInfo.nonIndexedNonIndexedPosition.end(),
                                                     "Repeated hessian elements requested")
                                loopInfo.nonIndexedNonIndexedPosition[orig] = e;
                            }

                            size_t tapeK = loop->getTempIndepIndexes(k)->tape;
                            loopInfo.equationGroups[g].nonIndexedTempEvals[orig].insert(k);
                            loopInfo.equationGroups[g].evalHessSparsity[posJ1->tape].insert(tapeK);
                        }

                    }
                }
            }

            /**
             * temporaries
             */
            if (_funNoLoops != nullptr) {
                const std::vector<set<size_t> >& gJac = _funNoLoops->getJacobianSparsity();
                size_t nk = _funNoLoops->getTemporaryDependentCount();
                size_t nOrigEq = _funNoLoops->getTapeDependentCount() - nk;

                const std::vector<set<size_t> >& dzdxx = _funNoLoops->getHessianTempEqsSparsity();

                std::vector<std::set<size_t> > usedTapeJ2(nEqGroups);

                for (size_t k1 = 0; k1 < nk; k1++) {
                    if (gJac[nOrigEq + k1].find(j1) == gJac[nOrigEq + k1].end()) {
                        continue;
                    }

                    const LoopPosition* posK1 = loop->getTempIndepIndexes(k1);
                    if (posK1 == nullptr) {
                        continue;
                    }

                    /**
                     * temporary - indexed
                     * d     d f_i
                     * d x_l d z_k1
                     */

                    for (size_t g = 0; g < nEqGroups; g++) {
                        const IterEquationGroup<Base>& group = eqGroups[g];
                        const std::vector<set<size_t> >& groupHess = group.getHessianSparsity();
                        HessianWithLoopsEquationGroupInfo<Base>& groupHessInfo = loopInfo.equationGroups[g];

                        const map<size_t, set<size_t> >& tapeJ22Iter = group.getHessianTempIndexedTapeIndexes(k1, j2);
                        for (const auto& ittj22iter : tapeJ22Iter) {
                            size_t tapeJ2 = ittj22iter.first;
                            const set<size_t>& iterations = ittj22iter.second;

                            bool used = usedTapeJ2[g].find(tapeJ2) != usedTapeJ2[g].end();
                            bool added = false;

                            for (size_t iteration : iterations) {
                                std::vector<HessianElement>* positions = nullptr;

                                bool flip = flipIndices2(groupHess, posK1->tape, tapeJ2);
                                if (flip) {
                                    if (!used) {
                                        pairss pos(tapeJ2, j1);
                                        positions = &groupHessInfo.indexedTempPositions[pos];
                                    }
                                    groupHessInfo.evalHessSparsity[tapeJ2].insert(posK1->tape);
                                } else {
                                    if (!used) {
                                        pairss pos(j1, tapeJ2);
                                        positions = &groupHessInfo.tempIndexedPositions[pos];
                                    }
                                    groupHessInfo.evalHessSparsity[posK1->tape].insert(tapeJ2);
                                }

                                if (positions != nullptr) {
                                    positions->resize(nIter);

                                    (*positions)[iteration].location = e;
                                    (*positions)[iteration].row = j1;
                                    (*positions)[iteration].count++;
                                    usedTapeJ2[g].insert(tapeJ2);
                                }

                                if (!added) {
                                    added = true;
                                    set<size_t>& evals = groupHessInfo.indexedTempEvals[pairss(tapeJ2, j1)];
                                    evals.insert(k1);
                                }
                            }

                        }

                        /**
                         * temporary - non-indexed
                         * d      d f_i
                         * d x_j2 d z_k1
                         */
                        if (posJ2 != nullptr) {
                            const set<size_t>& hessRow = group.getHessianSparsity()[posK1->tape];

                            if (hessRow.find(j2) != hessRow.end()) {
                                if (!jInNonIndexed) {
                                    jInNonIndexed = true;
                                    CPPADCG_ASSERT_KNOWN(loopInfo.nonIndexedNonIndexedPosition.find(orig) == loopInfo.nonIndexedNonIndexedPosition.end(),
                                                         "Repeated hessian elements requested")
                                    loopInfo.nonIndexedNonIndexedPosition[orig] = e;
                                }

                                groupHessInfo.tempNonIndexedEvals[orig].insert(k1);
                                groupHessInfo.evalHessSparsity[posK1->tape].insert(posJ2->tape);
                            }
                        }


                        /**
                         * temporary - temporary
                         *    d  d f_i    .  d z_k2
                         * d z_k2 d z_k1     d x_j2
                         */
                        // loop Hessian row
                        const set<size_t>& hessRow = group.getHessianSparsity()[posK1->tape];
                        auto itTapeJ2 = hessRow.lower_bound(nIndexed + nNonIndexed);
                        for (; itTapeJ2 != hessRow.end(); ++itTapeJ2) {
                            size_t tapeK2 = *itTapeJ2;
                            size_t k2 = loop->getTemporaryIndependents()[tapeK2 - nIndexed - nNonIndexed].original;

                            const set<size_t>& jacZk2Row = gJac[nOrigEq + k2];
                            if (jacZk2Row.find(j2) != jacZk2Row.end()) { // is this check truly needed?

                                if (!jInNonIndexed) {
                                    jInNonIndexed = true;
                                    CPPADCG_ASSERT_KNOWN(loopInfo.nonIndexedNonIndexedPosition.find(orig) == loopInfo.nonIndexedNonIndexedPosition.end(),
                                                         "Repeated hessian elements requested")
                                    loopInfo.nonIndexedNonIndexedPosition[orig] = e;
                                }

                                groupHessInfo.tempTempEvals[orig][k1].insert(k2);
                                groupHessInfo.evalHessSparsity[posK1->tape].insert(tapeK2);

                                noLoopEvalJacSparsity[nOrigEq + k2].insert(j2); // @todo: repeated operation for each group (only one needed)
                            }
                        }
                    }

                    //
                    noLoopEvalJacSparsity[nOrigEq + k1].insert(j1);

                    /**
                     * temporary - temporary
                     * d f_i   .  d      d z_k1
                     * d z_k1     d x_j2 d x_j1
                     */
                    if (dzdxx[j1].find(j2) != dzdxx[j1].end()) {

                        for (size_t i = 0; i < loopJac.size(); i++) {
                            const set<size_t>& fJacRow = loopJac[i];

                            if (fJacRow.find(posK1->tape) != fJacRow.end()) {
                                if (!jInNonIndexed) {
                                    CPPADCG_ASSERT_KNOWN(loopInfo.nonIndexedNonIndexedPosition.find(orig) == loopInfo.nonIndexedNonIndexedPosition.end(),
                                                         "Repeated hessian elements requested")

                                    loopInfo.nonIndexedNonIndexedPosition[orig] = e;
                                    jInNonIndexed = true;
                                }

                                loopInfo.nonLoopNonIndexedNonIndexed[orig].insert(k1);
                                loopInfo.evalJacSparsity[i].insert(posK1->tape);
                                loopInfo.noLoopEvalHessTempsSparsity[j1].insert(j2);
                            }
                        }
                    }
                }
            }

        }
    }
}

template<class Base>
inline void addContribution(std::vector<std::pair<CG<Base>, IndexPattern*> >& indexedLoopResults,
                            size_t& hessLE,
                            const std::pair<CG<Base>, IndexPattern*>& val) {
    if (!val.first.isIdenticalZero()) {
        if (indexedLoopResults.size() <= hessLE) {
            if (indexedLoopResults.capacity() <= hessLE) {
                indexedLoopResults.reserve(3 * hessLE / 2 + 1);
            }
            indexedLoopResults.resize(hessLE + 1);
        }
        indexedLoopResults[hessLE++] = val;
    }
}

template<class Base>
std::vector<CG<Base> > ModelCSourceGen<Base>::prepareSparseHessianWithLoops(CodeHandler<Base>& handler,
                                                                            std::vector<CGBase>& x,
                                                                            std::vector<CGBase>& w,
                                                                            const std::vector<size_t>& lowerHessRows,
                                                                            const std::vector<size_t>& lowerHessCols,
                                                                            const std::vector<size_t>& lowerHessOrder,
                                                                            const std::map<size_t, size_t>& duplicates) {
    using namespace std;
    using namespace CppAD::cg::loops;

    handler.setZeroDependents(true);

    size_t nonIndexdedEqSize = _funNoLoops != nullptr ? _funNoLoops->getOrigDependentIndexes().size() : 0;

    size_t maxLoc = _hessSparsity.rows.size();
    std::vector<CGBase> hess(maxLoc);

    std::vector<set<size_t> > noLoopEvalJacSparsity;
    std::vector<set<size_t> > noLoopEvalHessSparsity;
    std::vector<map<size_t, set<size_t> > > noLoopEvalHessLocations;
    map<LoopModel<Base>*, HessianWithLoopsInfo<Base> > loopHessInfo;

    /** 
     * Load locations in the compressed Hessian
     * d      d y_i
     * d x_j2 d x_j1
     */
    analyseSparseHessianWithLoops(lowerHessRows, lowerHessCols, lowerHessOrder,
                                  noLoopEvalJacSparsity, noLoopEvalHessSparsity,
                                  noLoopEvalHessLocations, loopHessInfo, true);

    /***********************************************************************
     *        generate the operation graph
     **********************************************************************/
    /**
     * prepare loop independents
     */
    OperationNode<Base>* iterationIndexDcl = handler.makeIndexDclrNode(LoopModel<Base>::ITERATION_INDEX_NAME);

    // temporaries (zero order)
    std::vector<CGBase> tmpsAlias;
    if (_funNoLoops != nullptr) {
        ADFun<CGBase>& fun = _funNoLoops->getTape();

        tmpsAlias.resize(fun.Range() - nonIndexdedEqSize);
        for (size_t k = 0; k < tmpsAlias.size(); k++) {
            // to be defined later
            tmpsAlias[k] = CG<Base>(*handler.makeNode(CGOpCode::Alias));
        }
    }

    /**
     * prepare loop independents
     */
    typename map<LoopModel<Base>*, HessianWithLoopsInfo<Base> >::iterator itLoop2Info;
    for (itLoop2Info = loopHessInfo.begin(); itLoop2Info != loopHessInfo.end(); ++itLoop2Info) {
        LoopModel<Base>& lModel = *itLoop2Info->first;
        HessianWithLoopsInfo<Base>& info = itLoop2Info->second;

        /**
         * make the loop start
         */
        info.loopStart = handler.makeLoopStartNode(*iterationIndexDcl, lModel.getIterationCount());

        info.iterationIndexOp = handler.makeIndexNode(*info.loopStart);
        set<IndexOperationNode<Base>*> indexesOps;
        indexesOps.insert(info.iterationIndexOp);

        /**
         * make the loop's indexed variables
         */
        std::vector<CGBase> indexedIndeps = createIndexedIndependents(handler, lModel, *info.iterationIndexOp);
        info.x = createLoopIndependentVector(handler, lModel, indexedIndeps, x, tmpsAlias);

        info.w = createLoopDependentVector(handler, lModel, *info.iterationIndexOp);
    }

    /**
     * Calculate Hessians and Jacobians
     */
    /**
     * Loops - evaluate Jacobian and Hessian
     */
    for (itLoop2Info = loopHessInfo.begin(); itLoop2Info != loopHessInfo.end(); ++itLoop2Info) {
        LoopModel<Base>& lModel = *itLoop2Info->first;
        HessianWithLoopsInfo<Base>& info = itLoop2Info->second;

        _cache.str("");
        _cache << "model (Jacobian + Hessian, loop " << lModel.getLoopId() << ")";
        std::string jobName = _cache.str();
        _cache.str("");
        startingJob("'" + jobName + "'", JobTimer::GRAPH);

        bool individualColoring = lModel.isContainsAtomics();
        info.evalLoopModelJacobianHessian(individualColoring);

        finishedJob();
    }

    /**
     * No loops
     */
    // Jacobian for temporaries
    map<size_t, map<size_t, CGBase> > dzDx;

    if (_funNoLoops != nullptr) {
        ADFun<CGBase>& fun = _funNoLoops->getTape();
        std::vector<CGBase> yNL(fun.Range());

        /**
         * Jacobian and Hessian - temporary variables
         */
        startingJob("'model (Jacobian + Hessian, temporaries)'", JobTimer::GRAPH);

        dzDx = _funNoLoops->calculateJacobianHessianUsedByLoops(handler,
                                                                loopHessInfo, x, yNL,
                                                                noLoopEvalJacSparsity,
                                                                false);

        finishedJob();

        for (size_t i = 0; i < tmpsAlias.size(); i++)
            tmpsAlias[i].getOperationNode()->getArguments().push_back(asArgument(yNL[nonIndexdedEqSize + i]));

        for (itLoop2Info = loopHessInfo.begin(); itLoop2Info != loopHessInfo.end(); ++itLoop2Info) {
            HessianWithLoopsInfo<Base>& info = itLoop2Info->second;
            // not needed anymore:
            info.dyiDzk.clear();
        }

        /**
         * Hessian - original equations
         */
        _funNoLoops->calculateHessian4OrignalEquations(x, w,
                                                       noLoopEvalHessSparsity, noLoopEvalHessLocations,
                                                       hess);
    }

    /**
     * Loops - Hessian
     */
    for (itLoop2Info = loopHessInfo.begin(); itLoop2Info != loopHessInfo.end(); ++itLoop2Info) {
        LoopModel<Base>& lModel = *itLoop2Info->first;
        size_t nIterations = lModel.getIterationCount();
        HessianWithLoopsInfo<Base>& info = itLoop2Info->second;

        // store results in indexedLoopResults
        size_t hessElSize = info.nonIndexedNonIndexedPosition.size();
        for (size_t g = 0; g < info.equationGroups.size(); g++) {
            HessianWithLoopsEquationGroupInfo<Base>& infog = info.equationGroups[g];
            hessElSize += infog.indexedIndexedPositions.size() +
                    infog.indexedTempPositions.size() +
                    infog.nonIndexedIndexedPositions.size();
        }

        if (hessElSize == 0)
            continue; // no second order information

        std::vector<pair<CGBase, IndexPattern*> > indexedLoopResults(hessElSize);
        size_t hessLE = 0;

        /**
         * loop the groups of equations present at the same iterations
         */
        for (size_t g = 0; g < info.equationGroups.size(); g++) {

            HessianWithLoopsEquationGroupInfo<Base>& infog = info.equationGroups[g];

            /*******************************************************************
             * indexed - indexed
             */
            for (const auto& it : infog.indexedIndexedPositions) {
                size_t tapeJ1 = it.first.first;
                size_t tapeJ2 = it.first.second;
                const std::vector<HessianElement>& positions = it.second;

                addContribution(indexedLoopResults, hessLE,
                                createHessianContribution(handler, positions, infog.hess[tapeJ1].at(tapeJ2),
                                                          *info.iterationIndexOp, info.ifElses));
            }

            /**
             * indexed - non-indexed
             * - usually done by  (non-indexed - indexed) by exploiting the symmetry
             */
            for (const auto& it : infog.indexedNonIndexedPositions) {
                size_t tapeJ1 = it.first.first;
                size_t tapeJ2 = it.first.second;
                const std::vector<HessianElement>& positions = it.second;

                addContribution(indexedLoopResults, hessLE,
                                createHessianContribution(handler, positions, infog.hess[tapeJ1].at(tapeJ2),
                                                          *info.iterationIndexOp, info.ifElses));
            }

            /**
             * indexed - temporary
             */
            if (!infog.indexedTempPositions.empty()) {
                for (const auto& itEval : infog.indexedTempEvals) {
                    size_t tapeJ1 = itEval.first.first;
                    size_t j2 = itEval.first.second;
                    const set<size_t>& ks = itEval.second;

                    const auto itPos = infog.indexedTempPositions.find(itEval.first);
                    if (itPos != infog.indexedTempPositions.end()) {
                        const std::vector<HessianElement>& positions = itPos->second;

                        CGBase hessVal = Base(0);
                        for (size_t k : ks) {
                            size_t tapeK = lModel.getTempIndepIndexes(k)->tape;
                            hessVal += infog.hess[tapeJ1].at(tapeK) * dzDx[k][j2];
                        }

                        addContribution(indexedLoopResults, hessLE,
                                        createHessianContribution(handler, positions, hessVal,
                                                                  *info.iterationIndexOp, info.ifElses));
                    }
                }
            }

            /*******************************************************************
             * non-indexed - indexed
             */
            for (const auto& it : infog.nonIndexedIndexedPositions) {
                size_t tapeJ1 = it.first.first;
                size_t tapeJ2 = it.first.second;
                const std::vector<HessianElement>& positions = it.second;

                addContribution(indexedLoopResults, hessLE,
                                createHessianContribution(handler, positions, infog.hess[tapeJ1].at(tapeJ2),
                                                          *info.iterationIndexOp, info.ifElses));
            }

            /*******************************************************************
             * temporary - indexed
             * 
             *      d f_i       .    d x_k1
             * d x_l2  d z_k1        d x_j1
             * 
             * -> usually done by  (indexed - temporary) by exploiting the symmetry
             */
            if (!infog.tempIndexedPositions.empty()) {
                for (const auto& itEval : infog.indexedTempEvals) {
                    size_t tapeJ2 = itEval.first.first;
                    size_t j1 = itEval.first.second;
                    const set<size_t>& ks = itEval.second;

                    const auto itPos = infog.tempIndexedPositions.find(pairss(j1, tapeJ2));
                    if (itPos != infog.tempIndexedPositions.end()) {
                        const std::vector<HessianElement>& positions = itPos->second;
                        CGBase hessVal = Base(0);
                        for (size_t k : ks) {
                            size_t tapeK = lModel.getTempIndepIndexes(k)->tape;
                            hessVal += infog.hess[tapeK].at(tapeJ2) * dzDx[k][j1];
                        }

                        addContribution(indexedLoopResults, hessLE,
                                        createHessianContribution(handler, positions, hessVal,
                                                                  *info.iterationIndexOp, info.ifElses));
                    }
                }
            }

        }

        /*******************************************************************
         * contributions to a constant location
         */
        for (const auto& orig2PosIt : info.nonIndexedNonIndexedPosition) {
            const pairss& orig = orig2PosIt.first;
            size_t e = orig2PosIt.second;

            size_t j1 = orig.first;
            size_t j2 = orig.second;
            const LoopPosition* posJ1 = lModel.getNonIndexedIndepIndexes(j1);
            const LoopPosition* posJ2 = lModel.getNonIndexedIndepIndexes(j2);

            // location
            LinearIndexPattern* pattern = new LinearIndexPattern(0, 0, 0, e);
            handler.manageLoopDependentIndexPattern(pattern);

            /**
             * non-indexed - non-indexed
             */
            CGBase hessVal = Base(0);

            /**
             * loop the groups of equations present at the same iterations
             */
            for (size_t g = 0; g < info.equationGroups.size(); g++) {
                const IterEquationGroup<Base>& group = lModel.getEquationsGroups()[g];

                CGBase gHessVal = Base(0);
                HessianWithLoopsEquationGroupInfo<Base>& infog = info.equationGroups[g];

                if (infog.nonIndexedNonIndexedEvals.find(orig) != infog.nonIndexedNonIndexedEvals.end()) {
                    gHessVal = infog.hess[posJ1->tape].at(posJ2->tape);
                }

                /**
                 * non-indexed - temporary
                 */
                const auto itNT = infog.nonIndexedTempEvals.find(orig);
                if (itNT != infog.nonIndexedTempEvals.end()) {
                    const set<size_t>& ks = itNT->second;

                    for (size_t k : ks) {
                        size_t tapeK = lModel.getTempIndepIndexes(k)->tape;
                        gHessVal += infog.hess[posJ1->tape].at(tapeK) * dzDx[k][j2];
                    }
                }

                /**
                 * temporary - non-indexed 
                 * 
                 *      d f_i       .    d x_k1
                 * d x_j2  d z_k1        d x_j1
                 */
                const auto itTN = infog.tempNonIndexedEvals.find(orig);
                if (itTN != infog.tempNonIndexedEvals.end()) {
                    const set<size_t>& ks = itTN->second;

                    for (size_t k1 : ks) {
                        size_t tapeK = lModel.getTempIndepIndexes(k1)->tape;
                        gHessVal += infog.hess[tapeK].at(posJ2->tape) * dzDx[k1][j1];
                    }
                }

                /**
                 * temporary - temporary
                 */
                const auto itTT = infog.tempTempEvals.find(orig);
                if (itTT != infog.tempTempEvals.end()) {
                    const map<size_t, set<size_t> >& k1k2 = itTT->second;

                    CGBase sum = Base(0);

                    for (const auto& itzz : k1k2) {
                        size_t k1 = itzz.first;
                        const set<size_t>& k2s = itzz.second;
                        size_t tapeK1 = lModel.getTempIndepIndexes(k1)->tape;

                        CGBase tmp = Base(0);
                        for (size_t k2 : k2s) {
                            size_t tapeK2 = lModel.getTempIndepIndexes(k2)->tape;

                            tmp += infog.hess[tapeK1].at(tapeK2) * dzDx[k2][j2];
                        }

                        sum += tmp * dzDx[k1][j1];
                    }

                    gHessVal += sum;
                }

                if (group.iterations.size() != nIterations) {
                    CGBase v = createHessianContribution(handler, *pattern, group.iterations,
                                                         nIterations, gHessVal,
                                                         *info.iterationIndexOp, info.ifElses);
                    addContribution(indexedLoopResults, hessLE, make_pair(v, (IndexPattern*) nullptr));
                } else {
                    hessVal += gHessVal;
                }

            }

            /**
             * temporary - temporary
             */
            const auto itTT2 = info.nonLoopNonIndexedNonIndexed.find(orig);
            if (itTT2 != info.nonLoopNonIndexedNonIndexed.end()) {
                hessVal += info.dzDxx.at(j1).at(j2); // it is already the sum of ddz / dx_j1 dx_j2
            }

            // place the result
            addContribution(indexedLoopResults, hessLE, make_pair(hessVal, (IndexPattern*) pattern));
        }

        indexedLoopResults.resize(hessLE);

        /**
         * make the loop end
         */
        size_t assignOrAdd = 1;
        set<IndexOperationNode<Base>*> indexesOps;
        indexesOps.insert(info.iterationIndexOp);
        info.loopEnd = createLoopEnd(handler, *info.loopStart, indexedLoopResults, indexesOps, assignOrAdd);

        for (size_t e : lowerHessOrder) {
            // an additional alias variable is required so that each dependent variable can have its own ID
            if (hess[e].isIdenticalZero()) {
                hess[e] = CG<Base>(*handler.makeNode(CGOpCode::DependentMultiAssign, *info.loopEnd));

            } else if (hess[e].getOperationNode() != nullptr && hess[e].getOperationNode()->getOperationType() == CGOpCode::DependentMultiAssign) {
                hess[e].getOperationNode()->getArguments().push_back(*info.loopEnd);

            } else {
                hess[e] = handler.createCG(*handler.makeNode(CGOpCode::DependentMultiAssign,{asArgument(hess[e]), *info.loopEnd}));
            }
        }

        // not needed anymore:
        for (size_t g = 0; g < info.equationGroups.size(); g++) {
            info.equationGroups[g].hess.clear();
        }
        info.dzDxx.clear();

        /**
         * move non-indexed expressions outside loop
         */
        moveNonIndexedOutsideLoop(handler, *info.loopStart, *info.loopEnd);
    }

    /**
     * duplicates (TODO: use loops)
     */
    // make use of the symmetry of the Hessian in order to reduce operations
    for (const auto& it2 : duplicates) {
        if (hess[it2.second].isVariable())
            hess[it2.first] = CG<Base>(*handler.makeNode(CGOpCode::Alias, asArgument(hess[it2.second])));
        else
            hess[it2.first] = hess[it2.second].getValue();
    }

    return hess;
}

namespace loops {

template<class Base>
std::pair<CG<Base>, IndexPattern*> createHessianContribution(CodeHandler<Base>& handler,
                                                             const std::vector<HessianElement>& positions,
                                                             const CG<Base>& ddfdxdx,
                                                             IndexOperationNode<Base>& iterationIndexOp,
                                                             std::vector<IfElseInfo<Base> >& ifElses) {
    using namespace std;

    if (ddfdxdx.isIdenticalZero()) {
        return make_pair(ddfdxdx, (IndexPattern*) nullptr);
    }

    // combine iterations with the same number of additions
    map<size_t, map<size_t, size_t> > locations;
    for (size_t iter = 0; iter < positions.size(); iter++) {
        size_t c = positions[iter].count;
        if (c > 0) {
            locations[c][iter] = positions[iter].location;
        }
    }

    map<size_t, CG<Base> > results;

    // generate the index pattern for the Hessian compressed element
    for (const auto& countIt : locations) {
        size_t count = countIt.first;

        CG<Base> val = ddfdxdx;
        for (size_t c = 1; c < count; c++)
            val += ddfdxdx;

        results[count] = val;
    }

    if (results.size() == 1 && locations.begin()->second.size() == positions.size()) {
        // same expression present in all iterations

        // generate the index pattern for the Hessian compressed element
        IndexPattern* pattern = IndexPattern::detect(locations.begin()->second);
        handler.manageLoopDependentIndexPattern(pattern);

        return make_pair(results.begin()->second, pattern);

    } else {
        /**
         * must create a conditional element so that this 
         * contribution to the Hessian is only evaluated at the
         * relevant iterations
         */
        map<size_t, IfBranchData<Base> > branches;

        // try to find an existing if-else where these operations can be added
        for (const auto& countIt : locations) {

            size_t count = countIt.first;
            IfBranchData<Base> branch(results[count], countIt.second);
            branches[count] = branch;
        }

        CG<Base> v = createConditionalContribution(handler,
                                                   branches,
                                                   positions.size() - 1,
                                                   positions.size(),
                                                   iterationIndexOp,
                                                   ifElses);

        return make_pair(v, (IndexPattern*) nullptr);
    }

}

/**
 * Hessian contribution to a constant location
 */
template<class Base>
CG<Base> createHessianContribution(CodeHandler<Base>& handler,
                                   LinearIndexPattern& pattern,
                                   const std::set<size_t>& iterations,
                                   size_t nIterations,
                                   const CG<Base>& ddfdxdx,
                                   IndexOperationNode<Base>& iterationIndexOp,
                                   std::vector<IfElseInfo<Base> >& ifElses) {
    using namespace std;

    if (ddfdxdx.isIdenticalZero()) {
        return ddfdxdx;
    }

    CPPADCG_ASSERT_UNKNOWN(pattern.getLinearSlopeDy() == 0) // must be a constant index

    if (iterations.size() == nIterations) {
        // same expression present in all iterations
        return ddfdxdx;

    } else {

        /**
         * must create a conditional element so that this 
         * contribution to the Hessian is only evaluated at the
         * relevant iterations
         */
        return createConditionalContribution(handler, pattern,
                                             iterations, nIterations - 1,
                                             ddfdxdx, iterationIndexOp,
                                             ifElses);
    }
}

template<class Base>
inline void generateLoopForJacHes(ADFun<CG<Base> >& fun,
                                  const std::vector<CG<Base> >& x,
                                  const std::vector<std::vector<CG<Base> > >& vw,
                                  std::vector<CG<Base> >& y,
                                  const std::vector<std::set<size_t> >& jacSparsity,
                                  const std::vector<std::set<size_t> >& jacEvalSparsity,
                                  std::vector<std::map<size_t, CG<Base> > >& jac,
                                  const std::vector<std::set<size_t> >& hesSparsity,
                                  const std::vector<std::set<size_t> >& hesEvalSparsity,
                                  std::vector<std::map<size_t, std::map<size_t, CG<Base> > > >& vhess,
                                  bool individualColoring) {
    using namespace std;

    using CGB = CG<Base>;

    size_t m = fun.Range();
    size_t n = fun.Domain();

    jac.resize(m);
    vhess.resize(vw.size());

    if (!individualColoring) {
        /**
         * No atomics
         */

        // Jacobian for temporaries
        std::vector<size_t> jacRow, jacCol;
        generateSparsityIndexes(jacEvalSparsity, jacRow, jacCol);

        // Jacobian for equations outside loops
        std::vector<CGB> jacFlat(jacRow.size());

        /**
         * Hessian - temporary variables
         */
        std::vector<size_t> hesRow, hesCol;
        generateSparsityIndexes(hesEvalSparsity, hesRow, hesCol);

        std::vector<std::vector<CGB> > vhessFlat(vw.size());
        for (size_t l = 0; l < vw.size(); l++) {
            vhessFlat[l].resize(hesRow.size());
        }

        std::vector<CG<Base> > xl;
        if (x.size() == 0) {
            xl.resize(1); // does not depend on any variable but CppAD requires at least one
            xl[0] = Base(0);
        } else {
            xl = x;
        }

        SparseForjacHessianWork work;
        sparseForJacHessian(fun, xl, vw,
                            y,
                            jacSparsity,
                            jacRow, jacCol, jacFlat,
                            hesSparsity,
                            hesRow, hesCol, vhessFlat,
                            work);

        // save Jacobian
        for (size_t el = 0; el < jacRow.size(); el++) {
            size_t i = jacRow[el];
            size_t j = jacCol[el];

            jac[i][j] = jacFlat[el];
        }

        // save Hessian
        for (size_t l = 0; l < vw.size(); l++) {
            std::vector<CGB>& hessFlat = vhessFlat[l];
            map<size_t, map<size_t, CGB> >& hess = vhess[l];

            for (size_t el = 0; el < hesRow.size(); el++) {
                size_t j1 = hesRow[el];
                size_t j2 = hesCol[el];
                hess[j1][j2] = hessFlat[el];
            }
        }

    } else {
        /**
         * Contains atomics
         */

        //transpose
        std::vector<set<size_t> > jacEvalSparsityT(n);
        addTransMatrixSparsity(jacEvalSparsity, jacEvalSparsityT);

        std::vector<CGB> tx1v(n);

        y = fun.Forward(0, x);

        for (size_t j1 = 0; j1 < n; j1++) {
            if (jacEvalSparsityT[j1].empty() && hesEvalSparsity[j1].empty()) {
                continue;
            }

            tx1v[j1] = Base(1);
            std::vector<CGB> dy = fun.Forward(1, tx1v);
            CPPADCG_ASSERT_UNKNOWN(dy.size() == m)
            tx1v[j1] = Base(0);

            // save Jacobian
            const set<size_t>& column = jacEvalSparsityT[j1];
            for (size_t i : column) {
                jac[i][j1] = dy[i];
            }

            const set<size_t>& hesRow = hesEvalSparsity[j1];

            if (!hesRow.empty()) {

                for (size_t l = 0; l < vw.size(); l++) {

                    std::vector<CGB> px = fun.Reverse(2, vw[l]);
                    CPPADCG_ASSERT_UNKNOWN(px.size() == 2 * n)

                    // save Hessian
                    map<size_t, CGB>& hessRow = vhess[l][j1];
                    for (size_t j2 : hesRow) {
                        hessRow[j2] = px[j2 * 2 + 1];
                    }
                }
            }
        }
    }

}

} // END loops namespace

} // END cg namespace
} // END CppAD namespace

#endif
