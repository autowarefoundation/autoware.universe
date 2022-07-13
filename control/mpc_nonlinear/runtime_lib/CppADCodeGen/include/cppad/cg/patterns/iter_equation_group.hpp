#ifndef CPPAD_CG_ITER_EQUATION_GROUP_INCLUDED
#define CPPAD_CG_ITER_EQUATION_GROUP_INCLUDED
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

/**
 * Group of equations present at the same iterations
 */
template <class Base>
class IterEquationGroup {
public:
    using CGB = CppAD::cg::CG<Base>;
    using Arg = Argument<Base>;
    using pairss = std::pair<size_t, size_t>;
private:
    static const std::vector<std::set<pairss> > EMPTYVECTORSETSS;
    static const std::vector<std::set<size_t> > EMPTYVECTORSETS;
    static const std::map<size_t, std::set<size_t> > EMPTYMAPSETS;
public:
    /// iteration group index/ID
    size_t index;
    /// equations indexes in tape of the loop model
    std::set<size_t> tapeI;
    /// iterations which only have these equations defined
    std::set<size_t> iterations;
    ///
    LoopModel<Base>* model;
private:
    /**
     * Hessian sparsity pattern of the tape
     */
    std::vector<std::set<size_t> > hessTapeSparsity_;
    bool hessSparsity_;
    /**
     * indexed Hessian elements
     * [{orig J1, orig J2}] -> [iteration -> [{tape J1, tape J2}]]
     */
    std::map<pairss, std::vector<std::set<pairss> > > hessOrig2Iter2TapeJ1TapeJ2_;
    /**
     * indexed Hessian elements
     * [{orig J1, orig J2}] -> [iteration -> [{tape J1}]]
     */
    std::map<pairss, std::vector<std::set<size_t> > > hessOrig2Iter2TapeJ1OrigJ2_;
    /**
     * non-indexed Hessian elements
     * [{orig J1, orig J2}] -> [iteration -> [{tape J2}]]
     */
    std::map<pairss, std::vector<std::set<size_t> > > hessOrig2Iter2OrigJ1TapeJ2_;
    /**
     * non-indexed Hessian elements
     * [{orig J1, orig J2}]
     */
    std::set<pairss> hessOrigJ1OrigJ2_;
    /**
     * temporary Hessian elements
     * [{k1, orig J2}] -> [tape J2 -> [{iteration }]]
     */
    std::map<pairss, std::map<size_t, std::set<size_t> > > hessOrig2TempTapeJ22Iter_;
public:

    inline IterEquationGroup() :
        index((std::numeric_limits<size_t>::max)()), // not really required
        model(nullptr),
        hessSparsity_(false) {
    }

    inline void evalHessianSparsity() {
        if (hessSparsity_) {
            return; // already evaluated
        }

        CPPADCG_ASSERT_UNKNOWN(model != nullptr);
        size_t iterationCount = model->getIterationCount();

        const std::vector<std::vector<LoopPosition> >& indexedIndepIndexes = model->getIndexedIndepIndexes();
        const std::vector<LoopPosition>& nonIndexedIndepIndexes = model->getNonIndexedIndepIndexes();
        const std::vector<LoopPosition>& temporaryIndependents = model->getTemporaryIndependents();

        ADFun<CGB>& fun = model->getTape();

        hessTapeSparsity_ = hessianSparsitySet<std::vector<std::set<size_t> >, CGB>(fun, tapeI);

        /**
         * make a database of the Hessian elements
         */
        size_t nIndexed = indexedIndepIndexes.size();
        size_t nNonIndexed = nonIndexedIndepIndexes.size();
        size_t nTemp = temporaryIndependents.size();

        for (size_t iter : iterations) {
            /**
             * indexed tapeJ1
             */
            for (size_t tapeJ1 = 0; tapeJ1 < nIndexed; tapeJ1++) {
                const std::set<size_t>& hessRow = hessTapeSparsity_[tapeJ1];
                size_t j1 = indexedIndepIndexes[tapeJ1][iter].original;

                std::set<size_t> ::const_iterator itTape2;
                for (itTape2 = hessRow.begin(); itTape2 != hessRow.end() && *itTape2 < nIndexed; ++itTape2) {
                    size_t j2 = indexedIndepIndexes[*itTape2][iter].original;
                    pairss orig(j1, j2);
                    pairss tapeTape(tapeJ1, *itTape2);
                    std::vector<std::set<pairss> >& iterations = hessOrig2Iter2TapeJ1TapeJ2_[orig];
                    iterations.resize(iterationCount);
                    iterations[iter].insert(tapeTape);
                }

                for (; itTape2 != hessRow.end() && *itTape2 < nIndexed + nNonIndexed; ++itTape2) {
                    size_t j2 = nonIndexedIndepIndexes[*itTape2 - nIndexed].original;
                    pairss orig(j1, j2);
                    std::vector<std::set<size_t> >& iterations = hessOrig2Iter2TapeJ1OrigJ2_[orig];
                    iterations.resize(iterationCount);
                    iterations[iter].insert(tapeJ1);
                }
            }

            /**
             * non-indexed tapeJ1
             */
            for (size_t tapeJ1 = nIndexed; tapeJ1 < nIndexed + nNonIndexed; tapeJ1++) {
                const std::set<size_t>& hessRow = hessTapeSparsity_[tapeJ1];
                size_t j1 = nonIndexedIndepIndexes[tapeJ1 - nIndexed].original;

                std::set<size_t> ::const_iterator itTape2;
                for (itTape2 = hessRow.begin(); itTape2 != hessRow.end() && *itTape2 < nIndexed; ++itTape2) {
                    size_t j2 = indexedIndepIndexes[*itTape2][iter].original;
                    pairss orig(j1, j2);
                    std::vector<std::set<size_t> >& iterations = hessOrig2Iter2OrigJ1TapeJ2_[orig];
                    iterations.resize(iterationCount);
                    iterations[iter].insert(*itTape2);
                }

                for (; itTape2 != hessRow.end() && *itTape2 < nIndexed + nNonIndexed; ++itTape2) {
                    size_t j2 = nonIndexedIndepIndexes[*itTape2 - nIndexed].original;
                    pairss orig(j1, j2);
                    hessOrigJ1OrigJ2_.insert(orig);
                }
            }

            /**
             * temporaries tapeJ1
             */
            for (size_t tapeJ1 = nIndexed + nNonIndexed; tapeJ1 < nIndexed + nNonIndexed + nTemp; tapeJ1++) {
                const std::set<size_t>& hessRow = hessTapeSparsity_[tapeJ1];
                size_t k1 = temporaryIndependents[tapeJ1 - nIndexed - nNonIndexed].original;

                std::set<size_t> ::const_iterator itTape2;
                for (itTape2 = hessRow.begin(); itTape2 != hessRow.end() && *itTape2 < nIndexed; ++itTape2) {
                    size_t j2 = indexedIndepIndexes[*itTape2][iter].original;
                    pairss pos(k1, j2);
                    std::map<size_t, std::set<size_t> >& var2iters = hessOrig2TempTapeJ22Iter_[pos];
                    var2iters[*itTape2].insert(iter);
                }

            }
        }

        hessSparsity_ = true;

    }

    inline const std::vector<std::set<size_t> >& getHessianSparsity() const {
        return hessTapeSparsity_;
    }

    /**
     *
     * @param origJ1
     * @param origJ2
     * @return maps each iteration to the pair of tape indexes present in
     *         the Hessian
     */
    inline const std::vector<std::set<pairss> >& getHessianIndexedIndexedTapeIndexes(size_t origJ1,
                                                                                     size_t origJ2) const {
        pairss orig(origJ1, origJ2);

        std::map<pairss, std::vector<std::set<pairss> > >::const_iterator it;
        it = hessOrig2Iter2TapeJ1TapeJ2_.find(orig);
        if (it != hessOrig2Iter2TapeJ1TapeJ2_.end()) {
            return it->second;
        } else {
            return EMPTYVECTORSETSS;
        }
    }

    inline const std::vector<std::set<size_t> >& getHessianIndexedNonIndexedTapeIndexes(size_t origJ1,
                                                                                        size_t origJ2) const {
        pairss orig(origJ1, origJ2);

        std::map<pairss, std::vector<std::set<size_t> > > ::const_iterator it;
        it = hessOrig2Iter2TapeJ1OrigJ2_.find(orig);
        if (it != hessOrig2Iter2TapeJ1OrigJ2_.end()) {
            return it->second;
        } else {
            return EMPTYVECTORSETS;
        }
    }

    inline const std::vector<std::set<size_t> >& getHessianNonIndexedIndexedTapeIndexes(size_t origJ1,
                                                                                        size_t origJ2) const {
        pairss orig(origJ1, origJ2);

        std::map<pairss, std::vector<std::set<size_t> > >::const_iterator it;
        it = hessOrig2Iter2OrigJ1TapeJ2_.find(orig);
        if (it != hessOrig2Iter2OrigJ1TapeJ2_.end()) {
            return it->second;
        } else {
            return EMPTYVECTORSETS;
        }
    }

    inline const std::set<std::pair<size_t, size_t> >& getHessianNonIndexedNonIndexedIndexes() const {
        return hessOrigJ1OrigJ2_;
    }

    inline const std::map<size_t, std::set<size_t> >& getHessianTempIndexedTapeIndexes(size_t k1,
                                                                                       size_t origJ2) const {
        pairss pos(k1, origJ2);

        std::map<pairss, std::map<size_t, std::set<size_t> > >::const_iterator it;
        it = hessOrig2TempTapeJ22Iter_.find(pos);
        if (it != hessOrig2TempTapeJ22Iter_.end()) {
            return it->second;
        } else {
            return EMPTYMAPSETS;
        }
    }

};

template<class Base>
const std::vector<std::set<std::pair<size_t, size_t> > > IterEquationGroup<Base>::EMPTYVECTORSETSS;

template<class Base>
const std::vector<std::set<size_t> > IterEquationGroup<Base>::EMPTYVECTORSETS;

template<class Base>
const std::map<size_t, std::set<size_t> > IterEquationGroup<Base>::EMPTYMAPSETS;

} // END cg namespace
} // END CppAD namespace

#endif
