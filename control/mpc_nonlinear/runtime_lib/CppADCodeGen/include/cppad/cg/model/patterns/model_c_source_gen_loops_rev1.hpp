#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_REV1_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_LOOPS_REV1_INCLUDED
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

/**
 * 
 * @param sources maps files names to the generated source code
 * @param elements  [equation]{vars}
 */
template<class Base>
void ModelCSourceGen<Base>::prepareSparseReverseOneWithLoops(const std::map<size_t, std::vector<size_t> >& elements) {
    using namespace std;
    using namespace CppAD::cg::loops;

    //printSparsityPattern(_jacSparsity.rows, _jacSparsity.cols, "jacobian", _fun.Range());

    size_t n = _fun.Domain();

    CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);
    handler.setZeroDependents(false);

    auto& indexJrowDcl = *handler.makeIndexDclrNode("jrow");
    auto& indexIterationDcl = *handler.makeIndexDclrNode(LoopModel<Base>::ITERATION_INDEX_NAME);
    auto& iterationIndexOp = *handler.makeIndexNode(indexIterationDcl);
    auto& jrowIndexOp = *handler.makeIndexNode(indexJrowDcl);

    std::vector<OperationNode<Base>*> localNodes(4);
    localNodes[0] = &indexJrowDcl;
    localNodes[1] = &indexIterationDcl;
    localNodes[2] = &iterationIndexOp;
    localNodes[3] = &jrowIndexOp;

    size_t nonIndexdedEqSize = _funNoLoops != nullptr ? _funNoLoops->getOrigDependentIndexes().size() : 0;

    std::vector<set<size_t> > noLoopEvalSparsity;
    std::vector<map<size_t, set<size_t> > > noLoopEvalLocations; // tape equation -> original J -> locations
    map<LoopModel<Base>*, std::vector<set<size_t> > > loopsEvalSparsities;
    map<LoopModel<Base>*, std::vector<JacobianWithLoopsRowInfo> > loopEqInfo;

    size_t nnz = _jacSparsity.rows.size();
    std::vector<size_t> rows(nnz);
    std::vector<size_t> cols(nnz);
    std::vector<size_t> locations(nnz);

    size_t p = 0;
    for (const auto& itI : elements) {//loop dependents/equations
        size_t i = itI.first;
        const std::vector<size_t>& r = itI.second;

        for (size_t e = 0; e < r.size(); e++) { // loop variables
            rows[p] = i;
            cols[p] = r[e];
            locations[p] = e;
            p++;
        }
    }
    CPPADCG_ASSERT_UNKNOWN(p == nnz);

    analyseSparseJacobianWithLoops(rows, cols, locations,
                                   noLoopEvalSparsity, noLoopEvalLocations, loopsEvalSparsities, loopEqInfo);

    std::vector<CGBase> x(n);
    handler.makeVariables(x);
    if (_x.size() > 0) {
        for (size_t i = 0; i < n; i++) {
            x[i].setValue(_x[i]);
        }
    }

    CGBase py;
    handler.makeVariable(py);
    if (_x.size() > 0) {
        py.setValue(Base(1.0));
    }

    /***********************************************************************
     *        generate the operation graph
     **********************************************************************/

    /**
     * original equations outside the loops 
     */
    // temporaries (zero orders)
    std::vector<CGBase> tmps;

    // jacobian for temporaries
    std::vector<map<size_t, CGBase> > dzDx(_funNoLoops != nullptr ? _funNoLoops->getTemporaryDependentCount() : 0);

    /*******************************************************************
     * equations NOT in loops
     ******************************************************************/
    if (_funNoLoops != nullptr) {
        ADFun<CGBase>& fun = _funNoLoops->getTape();

        /**
         * zero order
         */
        std::vector<CGBase> depNL = _funNoLoops->getTape().Forward(0, x);

        tmps.resize(depNL.size() - nonIndexdedEqSize);
        for (size_t i = 0; i < tmps.size(); i++)
            tmps[i] = depNL[nonIndexdedEqSize + i];

        /**
         * jacobian
         */
        bool hasAtomics = isAtomicsUsed(); // TODO: improve this by checking only the current fun
        std::vector<map<size_t, CGBase> > dydx = generateLoopRev1Jac(fun, _funNoLoops->getJacobianSparsity(), noLoopEvalSparsity, x, hasAtomics);

        const std::vector<size_t>& dependentIndexes = _funNoLoops->getOrigDependentIndexes();
        map<size_t, std::vector<CGBase> > jacNl; // by row

        for (size_t inl = 0; inl < nonIndexdedEqSize; inl++) {
            size_t i = dependentIndexes[inl];

            // prepare space for the jacobian of the original equations
            std::vector<CGBase>& row = jacNl[i];
            row.resize(dydx[inl].size());

            for (const auto& itjv : dydx[inl]) {
                size_t j = itjv.first;
                // (dy_i/dx_v) elements from equations outside loops
                const set<size_t>& locations = noLoopEvalLocations[inl][j];

                CPPADCG_ASSERT_UNKNOWN(locations.size() == 1); // one jacobian element should not be placed in several locations
                size_t e = *locations.begin();

                row[e] = itjv.second * py;

                _nonLoopRev1Elements[i].insert(e);
            }
        }

        // dz_k/dx_v (for temporary variable)
        for (size_t inl = nonIndexdedEqSize; inl < dydx.size(); inl++) {
            size_t k = inl - nonIndexdedEqSize;

            for (const auto& itjv : dydx[inl]) {
                size_t j = itjv.first;
                dzDx[k][j] = itjv.second;
            }
        }

        /**
         * Create source for each variable present in equations outside loops
         */
        typename map<size_t, std::vector<CGBase> >::iterator itJ;
        for (itJ = jacNl.begin(); itJ != jacNl.end(); ++itJ) {
            createReverseOneWithLoopsNL(handler, itJ->first, itJ->second);
        }
    }

    /***********************************************************************
     * equations in loops 
     **********************************************************************/
    typename map<LoopModel<Base>*, std::vector<JacobianWithLoopsRowInfo> >::iterator itl2Eq;
    for (itl2Eq = loopEqInfo.begin(); itl2Eq != loopEqInfo.end(); ++itl2Eq) {
        LoopModel<Base>& lModel = *itl2Eq->first;
        const std::vector<JacobianWithLoopsRowInfo>& info = itl2Eq->second;
        ADFun<CGBase>& fun = lModel.getTape();
        const std::vector<std::vector<LoopPosition> >& dependentIndexes = lModel.getDependentIndexes();
        size_t nIterations = lModel.getIterationCount();

        _cache.str("");
        _cache << "model (reverse one, loop " << lModel.getLoopId() << ")";
        std::string jobName = _cache.str();

        /**
         * evaluate loop model Jacobian
         */
        startingJob("'" + jobName + "'", JobTimer::GRAPH);

        std::vector<CGBase> indexedIndeps = createIndexedIndependents(handler, lModel, iterationIndexOp);
        std::vector<CGBase> xl = createLoopIndependentVector(handler, lModel, indexedIndeps, x, tmps);

        bool hasAtomics = isAtomicsUsed(); // TODO: improve this by checking only the current fun
        std::vector<map<size_t, CGBase> > dyiDxtape = generateLoopRev1Jac(fun, lModel.getJacobianSparsity(), loopsEvalSparsities[&lModel], xl, hasAtomics);

        finishedJob();

        /**
         * process each equation pattern (row in the loop tape)
         */
        std::vector<std::pair<CGBase, IndexPattern*> > indexedLoopResults;
        for (size_t tapeI = 0; tapeI < info.size(); tapeI++) {
            const JacobianWithLoopsRowInfo& rowInfo = info[tapeI];

            size_t maxRowEls = rowInfo.indexedPositions.size() + rowInfo.nonIndexedPositions.size();
            if (maxRowEls == 0)
                continue; // nothing to do (possibly an equation assigned to a constant value)

            /**
             * determine iteration index through the row index
             */
            map<size_t, size_t> irow2It;
            for (size_t it = 0; it < nIterations; it++) {
                size_t i = dependentIndexes[tapeI][it].original;
                if (i < _fun.Range()) // some equations are not present in all iteration
                    irow2It[i] = it;
            }

            std::unique_ptr<IndexPattern> itPattern(IndexPattern::detect(irow2It));
            auto* iterationIndexPatternOp = handler.makeIndexAssignNode(indexIterationDcl, *itPattern.get(), jrowIndexOp);
            iterationIndexOp.makeAssigmentDependent(*iterationIndexPatternOp);

            /**
             * generate the operation graph for this equation pattern
             */
            std::set<size_t> allLocations;
            indexedLoopResults.resize(maxRowEls);
            size_t jacLE = 0;

            std::vector<IfElseInfo<Base> > ifElses;

            prepareSparseJacobianRowWithLoops(handler, lModel,
                                              tapeI, rowInfo,
                                              dyiDxtape, dzDx,
                                              py,
                                              iterationIndexOp, ifElses,
                                              jacLE, indexedLoopResults, allLocations);
            indexedLoopResults.resize(jacLE);

            std::vector<CGBase> pxCustom(indexedLoopResults.size());

            for (size_t i = 0; i < indexedLoopResults.size(); i++) {
                const CGBase& val = indexedLoopResults[i].first;
                IndexPattern* ip = indexedLoopResults[i].second;

                pxCustom[i] = createLoopDependentFunctionResult(handler, i, val, ip, iterationIndexOp);
            }

            /**
             * save information on: row->{compressed reverse 1 position}
             */
            std::map<size_t, std::set<size_t> >& row2position = _loopRev1Groups[&lModel][tapeI];

            for (size_t it = 0; it < nIterations; it++) {
                size_t i = dependentIndexes[tapeI][it].original;
                if (i < _fun.Range()) { // some equations are not present in all iteration
                    std::set<size_t> positions;

                    for (const auto& itc : rowInfo.indexedPositions) {
                        const std::vector<size_t>& positionsC = itc.second;
                        if (positionsC[it] != (std::numeric_limits<size_t>::max)()) // not all elements are requested for all iterations
                            positions.insert(positionsC[it]);
                    }
                    for (const auto& itc : rowInfo.nonIndexedPositions) {
                        const std::vector<size_t>& positionsC = itc.second;
                        if (positionsC[it] != (std::numeric_limits<size_t>::max)()) // not all elements are requested for all iterations
                            positions.insert(positionsC[it]);
                    }

                    if (!positions.empty())
                        row2position[i].swap(positions);
                }
            }

            /**
             * Generate the source code
             */
            LanguageC<Base> langC(_baseTypeName);
            langC.setFunctionIndexArgument(indexJrowDcl);
            langC.setParameterPrecision(_parameterPrecision);

            _cache.str("");
            std::ostringstream code;
            std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator("dw"));
            LangCDefaultHessianVarNameGenerator<Base> nameGenHess(nameGen.get(), "dy", n);

            /**
             * Generate the source code inside the loop
             */
            _cache.str("");
            _cache << "model (reverse one, loop " << lModel.getLoopId() << ", group " << tapeI << ")";
            string jobName = _cache.str();
            handler.generateCode(code, langC, pxCustom, nameGenHess, _atomicFunctions, jobName);

            _cache.str("");
            generateFunctionNameLoopRev1(_cache, lModel, tapeI);
            std::string functionName = _cache.str();

            std::string argsDcl = langC.generateFunctionArgumentsDcl();

            _cache.str("");
            _cache << "#include <stdlib.h>\n"
                    "#include <math.h>\n"
                    "\n"
                    << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n"
                    "\n"
                    "void " << functionName << "(" << argsDcl << ") {\n";
            nameGenHess.customFunctionVariableDeclarations(_cache);
            _cache << langC.generateIndependentVariableDeclaration() << "\n";
            _cache << langC.generateDependentVariableDeclaration() << "\n";
            _cache << langC.generateTemporaryVariableDeclaration(false, false,
                                                                 handler.getExternalFuncMaxForwardOrder(),
                                                                 handler.getExternalFuncMaxReverseOrder()) << "\n";
            nameGenHess.prepareCustomFunctionVariables(_cache);

            // code inside the loop
            _cache << code.str();

            nameGenHess.finalizeCustomFunctionVariables(_cache);
            _cache << "}\n\n";

            _sources[functionName + ".c"] = _cache.str();
            _cache.str("");

            /**
             * prepare the nodes to be reused!
             */
            if (tapeI + 1 < info.size() || &lModel != loopEqInfo.rbegin()->first) {
                handler.resetNodes(); // uncolor nodes
            }
        }

    }

    /**
     * 
     */
    string functionRev1 = _name + "_" + FUNCTION_SPARSE_REVERSE_ONE;
    _sources[functionRev1 + ".c"] = generateGlobalForRevWithLoopsFunctionSource(elements,
                                                                                _loopRev1Groups, _nonLoopRev1Elements,
                                                                                functionRev1, _name, _baseTypeName, "dep",
                                                                                generateFunctionNameLoopRev1);
    /**
     * Sparsity
     */
    _cache.str("");
    generateSparsity1DSource2(_name + "_" + FUNCTION_REVERSE_ONE_SPARSITY, elements);
    _sources[_name + "_" + FUNCTION_REVERSE_ONE_SPARSITY + ".c"] = _cache.str();
    _cache.str("");
}

template<class Base>
void ModelCSourceGen<Base>::createReverseOneWithLoopsNL(CodeHandler<Base>& handler,
                                                        size_t i,
                                                        std::vector<CG<Base> >& jacRow) {
    size_t n = _fun.Domain();

    _cache.str("");
    _cache << "model (forward one, dep " << i << ") no loop";
    const std::string jobName = _cache.str();

    LanguageC<Base> langC(_baseTypeName);
    langC.setMaxAssignmentsPerFunction(_maxAssignPerFunc, &_sources);
    langC.setParameterPrecision(_parameterPrecision);
    _cache.str("");
    _cache << _name << "_" << FUNCTION_SPARSE_REVERSE_ONE << "_noloop_dep" << i;
    langC.setGenerateFunction(_cache.str());

    std::ostringstream code;
    std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator("dw"));
    LangCDefaultHessianVarNameGenerator<Base> nameGenHess(nameGen.get(), "dy", n);

    handler.generateCode(code, langC, jacRow, nameGenHess, _atomicFunctions, jobName);

    handler.resetNodes();
}

template<class Base>
std::vector<std::map<size_t, CG<Base> > > ModelCSourceGen<Base>::generateLoopRev1Jac(ADFun<CGBase>& fun,
                                                                                     const SparsitySetType& sparsity,
                                                                                     const SparsitySetType& evalSparsity,
                                                                                     const std::vector<CGBase>& x,
                                                                                     bool constainsAtomics) {
    using namespace std;

    size_t m = fun.Range();

    std::vector<map<size_t, CGBase> > dyDx(m);

    if (!constainsAtomics) {
        std::vector<size_t> row, col;
        generateSparsityIndexes(evalSparsity, row, col);

        if (row.size() == 0)
            return dyDx; // nothing to do

        std::vector<CGBase> jacLoop(row.size());

        CppAD::sparse_jacobian_work work; // temporary structure for CppAD
        fun.SparseJacobianReverse(x, sparsity, row, col, jacLoop, work);

        // organize results
        for (size_t el = 0; el < jacLoop.size(); el++) {
            size_t i = row[el];
            size_t j = col[el];
            dyDx[i][j] = jacLoop[el];
        }

    } else {

        std::vector<CGBase> w(m);

        for (size_t i = 0; i < m; i++) {
            const set<size_t>& row = evalSparsity[i];

            if (row.empty())
                continue;

            fun.Forward(0, x);

            w[i] = Base(1);
            std::vector<CGBase> dw = fun.Reverse(1, w);
            CPPADCG_ASSERT_UNKNOWN(dw.size() == fun.Domain());
            w[i] = Base(0);

            map<size_t, CGBase>& dyIDx = dyDx[i];

            for (size_t j : row) {
                dyIDx[j] = dw[j];
            }
        }

    }

    return dyDx;
}

template<class Base>
void ModelCSourceGen<Base>::generateFunctionNameLoopRev1(std::ostringstream& cache,
                                                         const LoopModel<Base>& loop,
                                                         size_t tapeI) {
    generateFunctionNameLoopRev1(cache, _name, loop, tapeI);
}

template<class Base>
void ModelCSourceGen<Base>::generateFunctionNameLoopRev1(std::ostringstream& cache,
                                                         const std::string& modelName,
                                                         const LoopModel<Base>& loop,
                                                         size_t tapeI) {
    cache << modelName << "_" << FUNCTION_SPARSE_REVERSE_ONE <<
            "_loop" << loop.getLoopId() << "_g" << tapeI;
}

} // END cg namespace
} // END CppAD namespace

#endif