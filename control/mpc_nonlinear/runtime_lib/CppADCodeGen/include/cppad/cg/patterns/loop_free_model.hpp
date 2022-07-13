#ifndef CPPAD_CG_LOOP_FREE_MODEL_INCLUDED
#define CPPAD_CG_LOOP_FREE_MODEL_INCLUDED
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

/**
 * Altered model without the loop equations and with extra dependents
 * for the non-indexed temporary variables used by loops
 *
 * @author Joao Leal
 */
template <class Base>
class LoopFreeModel {
public:
    using CGB = CppAD::cg::CG<Base>;
    using Arg = Argument<Base>;
    using VectorSet = std::vector<std::set<size_t> >;
protected:
    /**
     * The tape
     */
    ADFun<CGB> * const fun_;
    /**
     * The dependent variables in this tape to their original indexes
     */
    std::vector<size_t> dependentIndexes_;
    std::map<size_t, size_t> dependentOrig2Local;
    /**
     * Jacobian sparsity pattern of the tape
     */
    VectorSet jacTapeSparsity_;
    bool jacSparsity_;
    /**
     * Hessian sparsity pattern for equations used to determine the
     * temporaries (ignores the the original model equations)
     */
    VectorSet hessTapeTempSparsity_;
    /**
     * Hessian sparsity pattern for the original model equations in the tape
     * (ignores the equations for the temporaries)
     */
    VectorSet hessTapeOrigEqSparsity_;
    // whether or not the hessian sparsities have been evaluated
    bool hessSparsity_;
public:

    /**
     * Creates a model for the non-indexed operations
     *
     * @param fun
     * @param dependentOrigIndexes
     */
    LoopFreeModel(ADFun<CGB>* fun,
                  const std::vector<size_t>& dependentOrigIndexes) :
        fun_(fun),
        dependentIndexes_(dependentOrigIndexes),
        jacSparsity_(false),
        hessSparsity_(false) {
        CPPADCG_ASSERT_KNOWN(fun != nullptr, "fun cannot be null")
        CPPADCG_ASSERT_KNOWN(dependentOrigIndexes.size() <= fun->Range(), "invalid size")

        for (size_t il = 0; il < dependentIndexes_.size(); il++)
            dependentOrig2Local[dependentIndexes_[il]] = il;
    }

    LoopFreeModel(const LoopFreeModel<Base>&) = delete;
    LoopFreeModel& operator=(const LoopFreeModel<Base>&) = delete;

    inline ADFun<CGB>& getTape() const {
        return *fun_;
    }

    inline size_t getTapeDependentCount() const {
        return fun_->Range();
    }

    inline size_t getTemporaryDependentCount() const {
        return fun_->Range() - dependentIndexes_.size();
    }

    inline size_t getTapeIndependentCount() const {
        return fun_->Domain();
    }

    /**
     * Provides the dependent variables indexes present in the original
     * model
     */
    inline const std::vector<size_t>& getOrigDependentIndexes() const {
        return dependentIndexes_;
    }

    inline size_t getLocalDependentIndex(size_t origI) const {
        return dependentOrig2Local.at(origI);
    }

    inline void evalJacobianSparsity() {
        if (!jacSparsity_) {
            jacTapeSparsity_ = jacobianSparsitySet<VectorSet, CGB>(*fun_);
            jacSparsity_ = true;
        }
    }

    inline const VectorSet& getJacobianSparsity() const {
        return jacTapeSparsity_;
    }

    inline void evalHessianSparsity() {
        if (!hessSparsity_) {
            size_t mo = dependentIndexes_.size();
            size_t m = fun_->Range();
            size_t n = fun_->Domain();

            // hessian for the original equations
            std::set<size_t> eqs;
            if (mo != 0) {
                for (size_t i = 0; i < mo; i++)
                    eqs.insert(eqs.end(), i);

                hessTapeOrigEqSparsity_ = hessianSparsitySet<std::vector<std::set<size_t> >, CGB>(*fun_, eqs);
            }

            // hessian for the temporary variable equations
            if (m != mo) {
                eqs.clear();
                for (size_t i = mo; i < m; i++)
                    eqs.insert(eqs.end(), i);
                hessTapeTempSparsity_ = hessianSparsitySet<std::vector<std::set<size_t> >, CGB>(*fun_, eqs);
            } else {
                hessTapeTempSparsity_.resize(n);
            }

            hessSparsity_ = true;
        }
    }

    inline const VectorSet& getHessianTempEqsSparsity() const {
        CPPADCG_ASSERT_UNKNOWN(hessSparsity_)
        return hessTapeTempSparsity_;
    }

    inline const VectorSet& getHessianOrigEqsSparsity() const {
        CPPADCG_ASSERT_UNKNOWN(hessSparsity_)
        return hessTapeOrigEqSparsity_;
    }

    /**
     * Creates conditional nodes for temporary variables
     *
     * @param handler source code handler
     * @param iterations the iterations where the value should be evaluated
     * @param iterCount the number of iteration of the loop
     * @param value the value determined inside the loop
     * @param iterationIndexOp the iteration index operation for this loop
     * @return
     */
    inline CG<Base> createConditionalOperation(CodeHandler<Base>& handler,
                                               const std::set<size_t>& iterations,
                                               size_t iterCount,
                                               const CG<Base>& value,
                                               IndexOperationNode<Base>& iterationIndexOp) {
        using namespace std;

        if (iterations.size() == iterCount) {
            // present in all iterations

            return value;

        } else {

            // no point in creating branches where both branch sides are zero
            if (value.isIdenticalZero())
                return value;

            /**
             * must create a conditional element so that this
             * contribution is only evaluated at the relevant iterations
             */
            OperationNode<Base>* tmpDclVar = handler.makeNode(CGOpCode::TmpDcl);
            Argument<Base> tmpArg(*tmpDclVar);

            set<size_t> usedIter;
            OperationNode<Base>* cond = loops::createIndexConditionExpressionOp<Base>(handler, iterations, usedIter, iterCount - 1, iterationIndexOp);

            // if
            OperationNode<Base>* ifStart = handler.makeNode(CGOpCode::StartIf, *cond);

            OperationNode<Base>* tmpAssign1 = handler.makeNode(CGOpCode::LoopIndexedTmp,{tmpArg, asArgument(value)});
            OperationNode<Base>* ifAssign = handler.makeNode(CGOpCode::CondResult,{*ifStart, *tmpAssign1});

            // else
            OperationNode<Base>* elseStart = handler.makeNode(CGOpCode::Else,{*ifStart, *ifAssign});

            OperationNode<Base>* tmpAssign2 = handler.makeNode(CGOpCode::LoopIndexedTmp,{tmpArg, Base(0)});
            OperationNode<Base>* elseAssign = handler.makeNode(CGOpCode::CondResult,{*elseStart, *tmpAssign2});

            // end if
            OperationNode<Base>* endIf = handler.makeNode(CGOpCode::EndIf,{*elseStart, *elseAssign});

            //
            OperationNode<Base>* tmpVar = handler.makeNode(CGOpCode::Tmp,{tmpArg, *endIf});
            return CG<Base>(*tmpVar);
        }

    }

    /**
     * Determines the Hessian for the temporary variables only used by
     * each loop
     *
     * @param loopHessInfo
     * @param x the independent variables
     * @param temps
     * @param noLoopEvalJacSparsity
     * @param individualColoring
     * @param iterationIndexOp
     * @return
     */
    inline std::map<size_t, std::map<size_t, CGB> > calculateJacobianHessianUsedByLoops(CodeHandler<Base>& handler,
                                                                                        std::map<LoopModel<Base>*, loops::HessianWithLoopsInfo<Base> >& loopHessInfo,
                                                                                        const std::vector<CGB>& x,
                                                                                        std::vector<CGB>& temps,
                                                                                        const VectorSet& noLoopEvalJacSparsity,
                                                                                        bool individualColoring) {
        using namespace std;
        using namespace CppAD::cg::loops;

        CPPADCG_ASSERT_UNKNOWN(hessSparsity_) // check that the sparsities have been evaluated

        size_t mo = dependentIndexes_.size();
        size_t m = getTapeDependentCount();
        size_t n = fun_->Domain();

        std::vector<std::vector<CGB> > vwNoLoop(loopHessInfo.size());
        std::vector<map<size_t, map<size_t, CGB> > > vhessNoLoop(loopHessInfo.size());

        /**
         * Hessian - temporary variables
         */
        std::vector<std::set<size_t> > noLoopEvalHessTempsSparsity(n);

        for (const auto& itLoop2Info : loopHessInfo) {
            const HessianWithLoopsInfo<Base>& info = itLoop2Info.second;

            addMatrixSparsity(info.noLoopEvalHessTempsSparsity, noLoopEvalHessTempsSparsity);
        }
        std::vector<size_t> hesRow, hesCol;
        generateSparsityIndexes(noLoopEvalHessTempsSparsity, hesRow, hesCol);

        size_t l = 0;
        for (const auto& itLoop2Info : loopHessInfo) {
            LoopModel<Base>* loop = itLoop2Info.first;
            const HessianWithLoopsInfo<Base>& info = itLoop2Info.second;

            const std::vector<IterEquationGroup<Base> >& eqGroups = loop->getEquationsGroups();
            size_t nIterations = loop->getIterationCount();
            size_t nEqGroups = eqGroups.size();

            std::vector<CGB>& wNoLoop = vwNoLoop[l];
            wNoLoop.resize(m);
            for (size_t inl = 0; inl < mo; inl++) {
                wNoLoop[inl] = Base(0);
            }

            for (size_t inl = mo; inl < m; inl++) {
                size_t k = inl - mo;
                const LoopPosition* posK = loop->getTempIndepIndexes(k);

                if (posK != nullptr) {

                    for (size_t g = 0; g < nEqGroups; g++) {
                        const IterEquationGroup<Base>& group = eqGroups[g];

                        CGB v = Base(0);

                        for (size_t tapeI : group.tapeI) {
                            const map<size_t, CGB>& row = info.dyiDzk[tapeI];
                            typename map<size_t, CGB>::const_iterator itCol = row.find(posK->tape);
                            if (itCol != row.end()) {
                                const CGB& dydz = itCol->second;
                                v += dydz * info.w[tapeI];
                            }
                        }

                        /**
                         * Some equations are not present in all iterations
                         */
                        v = createConditionalOperation(handler,
                                                       group.iterations,
                                                       nIterations,
                                                       v,
                                                       *info.iterationIndexOp);

                        wNoLoop[inl] += v;
                    }

                }
            }

            l++;
        }

        std::vector<map<size_t, CGB> > dyDx;
        generateLoopForJacHes(*fun_, x, vwNoLoop, temps,
                              getJacobianSparsity(),
                              noLoopEvalJacSparsity,
                              dyDx,
                              hessTapeTempSparsity_,
                              noLoopEvalHessTempsSparsity,
                              vhessNoLoop,
                              individualColoring);

        // save Jacobian
        map<size_t, map<size_t, CGB> > dzDx;
        for (size_t inl = mo; inl < m; inl++) {
            // dz_k/dx_v (for temporary variable)
            size_t k = inl - mo;
            dzDx[k] = dyDx[inl];
        }

        // save Hessian
        l = 0;
        for (auto& itLoop2Info : loopHessInfo) {
            HessianWithLoopsInfo<Base>& info = itLoop2Info.second;
            info.dzDxx = vhessNoLoop[l];
            l++;
        }

        return dzDx;
    }

    inline void calculateHessian4OrignalEquations(const std::vector<CGB>& x,
                                                  const std::vector<CGB>& w,
                                                  const VectorSet& noLoopEvalHessSparsity,
                                                  const std::vector<std::map<size_t, std::set<size_t> > >& noLoopEvalHessLocations,
                                                  std::vector<CGB>& hess) {
        using namespace std;
        using namespace CppAD::cg::loops;

        CPPADCG_ASSERT_UNKNOWN(hessSparsity_) // check that the sparsities have been evaluated

        std::vector<CGB> wNoLoop(getTapeDependentCount());
        std::vector<CGB> hessNoLoop;

        /**
         * hessian - original equations
         */
        std::vector<size_t> row, col;
        generateSparsityIndexes(noLoopEvalHessSparsity, row, col);

        if (!row.empty()) {
            hessNoLoop.resize(row.size());

            for (size_t inl = 0; inl < dependentIndexes_.size(); inl++) {
                wNoLoop[inl] = w[dependentIndexes_[inl]];
            }

            CppAD::sparse_hessian_work work; // temporary structure for CPPAD
            // "cppad.symmetric" may have missing values for functions using
            // atomic functions which only provide half of the elements
            // (some values could be zeroed)
            work.color_method = "cppad.general";
            fun_->SparseHessian(x, wNoLoop, hessTapeOrigEqSparsity_, row, col, hessNoLoop, work);

            // save non-indexed hessian elements
            for (size_t el = 0; el < row.size(); el++) {
                size_t j1 = row[el];
                size_t j2 = col[el];
                const set<size_t>& locations = noLoopEvalHessLocations[j1].at(j2);
                for (size_t itE : locations)
                    hess[itE] = hessNoLoop[el];
            }
        }
    }

    virtual ~LoopFreeModel() {
        delete fun_;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
