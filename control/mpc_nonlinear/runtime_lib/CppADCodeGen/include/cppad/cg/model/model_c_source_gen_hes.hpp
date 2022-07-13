#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_HES_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_HES_INCLUDED
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
void ModelCSourceGen<Base>::generateHessianSource() {
    using std::vector;

    const std::string jobName = "Hessian";

    startingJob("'" + jobName + "'", JobTimer::GRAPH);

    CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);

    size_t m = _fun.Range();
    size_t n = _fun.Domain();


    // independent variables
    vector<CGBase> indVars(n);
    handler.makeVariables(indVars);
    if (_x.size() > 0) {
        for (size_t i = 0; i < n; i++) {
            indVars[i].setValue(_x[i]);
        }
    }

    // multipliers
    vector<CGBase> w(m);
    handler.makeVariables(w);
    if (_x.size() > 0) {
        for (size_t i = 0; i < m; i++) {
            w[i].setValue(Base(1.0));
        }
    }

    vector<CGBase> hess = _fun.Hessian(indVars, w);

    // make use of the symmetry of the Hessian in order to reduce operations
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < i; j++) {
            hess[i * n + j] = hess[j * n + i];
        }
    }

    finishedJob();

    LanguageC<Base> langC(_baseTypeName);
    langC.setMaxAssignmentsPerFunction(_maxAssignPerFunc, &_sources);
    langC.setMaxOperationsPerAssignment(_maxOperationsPerAssignment);
    langC.setParameterPrecision(_parameterPrecision);
    langC.setGenerateFunction(_name + "_" + FUNCTION_HESSIAN);

    std::ostringstream code;
    std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator("hess"));
    LangCDefaultHessianVarNameGenerator<Base> nameGenHess(nameGen.get(), n);

    handler.generateCode(code, langC, hess, nameGenHess, _atomicFunctions, jobName);
}

template<class Base>
void ModelCSourceGen<Base>::generateSparseHessianSource(MultiThreadingType multiThreadingType) {
    /**
     * Determine the sparsity pattern p for Hessian of w^T F
     */
    determineHessianSparsity();

    if (_sparseHessianReusesRev2 && _reverseTwo) {
        generateSparseHessianSourceFromRev2(multiThreadingType);
    } else {
        generateSparseHessianSourceDirectly();
    }
}

template<class Base>
void ModelCSourceGen<Base>::generateSparseHessianSourceDirectly() {
    using std::vector;

    const std::string jobName = "sparse Hessian";
    size_t m = _fun.Range();
    size_t n = _fun.Domain();

    /**
     * we might have to consider a slightly different order than the one
     * specified by the user according to the available elements in the sparsity
     */
    std::vector<size_t> evalRows, evalCols;
    determineSecondOrderElements4Eval(evalRows, evalCols);

    std::map<size_t, std::map<size_t, size_t> > locations;
    for (size_t e = 0; e < evalRows.size(); e++) {
        size_t j1 = evalRows[e];
        size_t j2 = evalCols[e];
        std::map<size_t, std::map<size_t, size_t> >::iterator itJ1 = locations.find(j1);
        if (itJ1 == locations.end()) {
            locations[j1][j2] = e;
        } else {
            std::map<size_t, size_t>& j22e = itJ1->second;
            if (j22e.find(j2) == j22e.end()) {
                j22e[j2] = e; // OK
            } else {
                // repeated elements not allowed
                throw CGException("Repeated Hessian element requested: ", j1, " ", j2);
            }
        }
    }

    // make use of the symmetry of the Hessian in order to reduce operations
    std::vector<size_t> lowerHessRows, lowerHessCols, lowerHessOrder;
    lowerHessRows.reserve(_hessSparsity.rows.size() / 2);
    lowerHessCols.reserve(lowerHessRows.size());
    lowerHessOrder.reserve(lowerHessRows.size());

    std::map<size_t, size_t> duplicates; // the elements determined using symmetry
    std::map<size_t, std::map<size_t, size_t> >::const_iterator itJ;
    std::map<size_t, size_t>::const_iterator itI;
    for (size_t e = 0; e < evalRows.size(); e++) {
        bool add = true;
        size_t i = evalRows[e];
        size_t j = evalCols[e];
        if (i < j) {
            // find the symmetric value
            itJ = locations.find(j);
            if (itJ != locations.end()) {
                itI = itJ->second.find(i);
                if (itI != itJ->second.end()) {
                    size_t eSim = itI->second;
                    duplicates[e] = eSim;
                    add = false; // symmetric value being determined
                }
            }
        }

        if (add) {
            lowerHessRows.push_back(i);
            lowerHessCols.push_back(j);
            lowerHessOrder.push_back(e);
        }
    }

    /**
     * 
     */
    startingJob("'" + jobName + "'", JobTimer::GRAPH);

    CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);

    // independent variables
    vector<CGBase> indVars(n);
    handler.makeVariables(indVars);
    if (_x.size() > 0) {
        for (size_t i = 0; i < n; i++) {
            indVars[i].setValue(_x[i]);
        }
    }

    // multipliers
    vector<CGBase> w(m);
    handler.makeVariables(w);
    if (_x.size() > 0) {
        for (size_t i = 0; i < m; i++) {
            w[i].setValue(Base(1.0));
        }
    }

    vector<CGBase> hess(_hessSparsity.rows.size());
    if (_loopTapes.empty()) {
        CppAD::sparse_hessian_work work;
        // "cppad.symmetric" may have missing values for functions using atomic 
        // functions which only provide half of the elements 
        // (some values could be zeroed)
        work.color_method = "cppad.general";
        vector<CGBase> lowerHess(lowerHessRows.size());
        _fun.SparseHessian(indVars, w, _hessSparsity.sparsity, lowerHessRows, lowerHessCols, lowerHess, work);

        for (size_t i = 0; i < lowerHessOrder.size(); i++) {
            hess[lowerHessOrder[i]] = lowerHess[i];
        }

        // make use of the symmetry of the Hessian in order to reduce operations
        for (const auto& it2 : duplicates) {
            hess[it2.first] = hess[it2.second];
        }
    } else {
        /**
         * with loops
         */
        hess = prepareSparseHessianWithLoops(handler, indVars, w,
                                             lowerHessRows, lowerHessCols, lowerHessOrder,
                                             duplicates);
    }

    finishedJob();

    LanguageC<Base> langC(_baseTypeName);
    langC.setMaxAssignmentsPerFunction(_maxAssignPerFunc, &_sources);
    langC.setMaxOperationsPerAssignment(_maxOperationsPerAssignment);
    langC.setParameterPrecision(_parameterPrecision);
    langC.setGenerateFunction(_name + "_" + FUNCTION_SPARSE_HESSIAN);

    std::ostringstream code;
    std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator("hess"));
    LangCDefaultHessianVarNameGenerator<Base> nameGenHess(nameGen.get(), n);

    handler.generateCode(code, langC, hess, nameGenHess, _atomicFunctions, jobName);
}

template<class Base>
void ModelCSourceGen<Base>::generateSparseHessianSourceFromRev2(MultiThreadingType multiThreadingType) {
    using namespace std;

    /**
     * we might have to consider a slightly different order than the one
     * specified by the user according to the available elements in the sparsity
     */
    std::vector<size_t> evalRows, evalCols;
    determineSecondOrderElements4Eval(evalRows, evalCols);

    std::map<size_t, CompressedVectorInfo> hessInfo;

    // elements[var]{var}
    for (size_t e = 0; e < evalRows.size(); e++) {
        hessInfo[evalRows[e]].indexes.push_back(evalCols[e]);
    }

    // maps each element to its position in the user hessian
    for (auto& it : hessInfo) {
        it.second.locations = determineOrderByRow(it.first, it.second.indexes, evalRows, evalCols);
    }

    /**
     * determine to which functions we can provide the hessian row directly
     * without needing a temporary array (compressed)
     */
    for (auto& it : hessInfo) {
        const std::vector<size_t>& els = it.second.indexes;
        const std::vector<set<size_t> >& location = it.second.locations;
        CPPADCG_ASSERT_UNKNOWN(els.size() == location.size());
        CPPADCG_ASSERT_UNKNOWN(els.size() > 0);

        bool passed = true;
        size_t hessRowStart = *location[0].begin();
        for (size_t e = 0; e < els.size(); e++) {
            if (location[e].size() > 1) {
                passed = false; // too many elements
                break;
            }
            if (*location[e].begin() != hessRowStart + e) {
                passed = false; // wrong order
                break;
            }
        }
        it.second.ordered = passed;
    }

    /**
     * determine the maximum size of the temporary array
     */
    size_t maxCompressedSize = 0;

    for (const auto& it : hessInfo) {
        if (it.second.indexes.size() > maxCompressedSize && !it.second.ordered)
            maxCompressedSize = it.second.indexes.size();
    }

    if (!_loopTapes.empty()) {
        /**
         * with loops
         */
        generateSparseHessianWithLoopsSourceFromRev2(hessInfo, maxCompressedSize);
        return;
    }

    string functionName = _name + "_" + FUNCTION_SPARSE_HESSIAN;
    string functionRev2 = _name + "_" + FUNCTION_SPARSE_REVERSE_TWO;
    string rev2Suffix = "indep";

    if (!_multiThreading || multiThreadingType == MultiThreadingType::NONE) {
        _sources[functionName + ".c"] = generateSparseHessianRev2SingleThreadSource(functionName, hessInfo, maxCompressedSize, functionRev2, rev2Suffix);
    } else {
        _sources[functionName + ".c"] = generateSparseHessianRev2MultiThreadSource(functionName, hessInfo, maxCompressedSize, functionRev2, rev2Suffix, multiThreadingType);
    }
    _cache.str("");
}

template<class Base>
std::string ModelCSourceGen<Base>::generateSparseHessianRev2SingleThreadSource(const std::string& functionName,
                                                                               std::map<size_t, CompressedVectorInfo> hessInfo,
                                                                               size_t maxCompressedSize,
                                                                               const std::string& functionRev2,
                                                                               const std::string& rev2Suffix) {
    LanguageC<Base> langC(_baseTypeName);
    std::string argsDcl = langC.generateDefaultFunctionArgumentsDcl();
    std::vector<std::string> argsDcl2 = langC.generateDefaultFunctionArgumentsDcl2();

    _cache.str("");
    _cache << "#include <stdlib.h>\n"
            << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n\n";
    generateFunctionDeclarationSource(_cache, functionRev2, rev2Suffix, hessInfo, argsDcl);
    _cache << "\n";
    LanguageC<Base>::printFunctionDeclaration(_cache, "void", functionName, argsDcl2);
    _cache << " {\n"
            "   " << _baseTypeName << " const * inLocal[3];\n"
            "   " << _baseTypeName << " inLocal1 = 1;\n"
            "   " << _baseTypeName << " * outLocal[1];\n";
    if (maxCompressedSize > 0) {
        _cache << "   " << _baseTypeName << " compressed[" << maxCompressedSize << "];\n";
    }
    _cache << "   " << _baseTypeName << " * hess = out[0];\n"
            "\n"
            "   inLocal[0] = in[0];\n"
            "   inLocal[1] = &inLocal1;\n"
            "   inLocal[2] = in[1];\n";
    if (maxCompressedSize > 0) {
        _cache << "   outLocal[0] = compressed;";
    }

    langC.setArgumentIn("inLocal");
    langC.setArgumentOut("outLocal");
    std::string argsLocal = langC.generateDefaultFunctionArguments();
    bool previousCompressed = true;
    for (auto& it : hessInfo) {
        size_t index = it.first;
        const std::vector<size_t>& els = it.second.indexes;
        const std::vector<std::set<size_t> >& location = it.second.locations;
        CPPADCG_ASSERT_UNKNOWN(els.size() == location.size());
        CPPADCG_ASSERT_UNKNOWN(els.size() > 0);

        _cache << "\n";
        bool compressed = !it.second.ordered;
        if (!compressed) {
            _cache << "   outLocal[0] = &hess[" << *location[0].begin() << "];\n";
        } else if (!previousCompressed) {
            _cache << "   outLocal[0] = compressed;\n";
        }
        _cache << "   " << functionRev2 << "_" << rev2Suffix << index << "(" << argsLocal << ");\n";
        if (compressed) {
            for (size_t e = 0; e < els.size(); e++) {
                _cache << "   ";
                for (size_t itl : location[e]) {
                    _cache << "hess[" << itl << "] = ";
                }
                _cache << "compressed[" << e << "];\n";
            }
        }
        previousCompressed = compressed;
    }

    _cache << "\n"
            "}\n";
    return _cache.str();
}


template<class Base>
std::string ModelCSourceGen<Base>::generateSparseHessianRev2MultiThreadSource(const std::string& functionName,
                                                                              std::map<size_t, CompressedVectorInfo> hessInfo,
                                                                              size_t maxCompressedSize,
                                                                              const std::string& functionRev2,
                                                                              const std::string& rev2Suffix,
                                                                              MultiThreadingType multiThreadingType) {
    CPPADCG_ASSERT_UNKNOWN(_multiThreading);
    CPPADCG_ASSERT_UNKNOWN(multiThreadingType != MultiThreadingType::NONE);

    LanguageC<Base> langC(_baseTypeName);
    std::string argsDcl = langC.generateDefaultFunctionArgumentsDcl();
    std::vector<std::string> argsDcl2 = langC.generateDefaultFunctionArgumentsDcl2();

    _cache.str("");
    _cache << "#include <stdlib.h>\n"
           << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n\n";
    generateFunctionDeclarationSource(_cache, functionRev2, rev2Suffix, hessInfo, argsDcl);


    langC.setArgumentIn("inLocal");
    langC.setArgumentOut("outLocal");
    std::string argsLocal = langC.generateDefaultFunctionArguments();

    /**
     * Create independent functions for each row/column of the Jacobian
     */
    for (const auto& it : hessInfo) {
        size_t index = it.first;
        const std::vector<size_t>& els = it.second.indexes;
        const std::vector<std::set<size_t> >& location = it.second.locations;
        CPPADCG_ASSERT_UNKNOWN(els.size() == location.size());

        bool compressed = !it.second.ordered;
        if (!compressed) {
            continue;
        }

        std::string functionNameWrap = functionRev2 + "_" + rev2Suffix + std::to_string(index) + "_wrap";
        LanguageC<Base>::printFunctionDeclaration(_cache, "void", functionNameWrap, argsDcl2);
        _cache << " {\n"
                "   " << _baseTypeName << " const * inLocal[3];\n"
                "   " << _baseTypeName << " inLocal1 = 1;\n"
                "   " << _baseTypeName << " * outLocal[1];\n"
                "   " << _baseTypeName << " compressed[" << it.second.indexes.size() << "];\n"
                "   " << _baseTypeName << " * hess = out[0];\n"
                "\n"
                "   inLocal[0] = in[0];\n"
                "   inLocal[1] = &inLocal1;\n"
                "   inLocal[2] = in[1];\n"
                "   outLocal[0] = compressed;\n";
        _cache << "   " << functionRev2 << "_" << rev2Suffix << index << "(" << argsLocal << ");\n";
        for (size_t e = 0; e < els.size(); e++) {
            _cache << "   ";
            for (size_t itl : location[e]) {
                _cache << "hess[" << itl << "] = ";
            }
            _cache << "compressed[" << e << "];\n";
        }
        _cache << "}\n";
    }

    _cache << "\n"
            "typedef void (*cppadcg_function_type) (" << argsDcl << ");\n";


    if (multiThreadingType == MultiThreadingType::OPENMP) {
        _cache << "\n";
        printFileStartOpenMP(_cache);
        _cache << "\n";

    } else {
        /**
         * PThreads pool needs a function with a void pointer argument
         */
        assert(multiThreadingType == MultiThreadingType::PTHREADS);

        printFileStartPThreads(_cache, _baseTypeName);
    }

    /**
     * Hessian function
     */
    _cache << "\n"
            "void " << functionName << "(" << argsDcl << ") {\n"
            "   static const cppadcg_function_type p[" << hessInfo.size() << "] = {";
    for (const auto& it : hessInfo) {
        size_t index = it.first;
        if (index != hessInfo.begin()->first) _cache << ", ";
        if (it.second.ordered) {
            _cache << functionRev2 << "_" << rev2Suffix << index;
        } else {
            _cache << functionRev2 << "_" << rev2Suffix << index << "_wrap";
        }
    }
    _cache << "};\n"
            "   static const long offset["<< hessInfo.size() <<"] = {";
    for (const auto& it : hessInfo) {
        if (it.first != hessInfo.begin()->first) _cache << ", ";
        if (it.second.ordered) {
            _cache << *it.second.locations[0].begin();
        } else {
            _cache << "0";
        }
    }
    _cache << "};\n"
            "   " << _baseTypeName << " inLocal1 = 1;\n"
            "   " << _baseTypeName << " const * inLocal[3] = {in[0], &inLocal1, in[1]};\n"
            "   " << _baseTypeName << " * outLocal[1];\n";
    _cache << "   " << _baseTypeName << " * hess = out[0];\n"
            "   long i;\n"
            "\n";

    if(multiThreadingType == MultiThreadingType::OPENMP) {
        printFunctionStartOpenMP(_cache, hessInfo.size());
        _cache << "\n";
        printLoopStartOpenMP(_cache, hessInfo.size());
        _cache << "      outLocal[0] = &hess[offset[i]];\n"
                "      (*p[i])(" << argsLocal << ");\n";
        printLoopEndOpenMP(_cache, hessInfo.size());
        _cache << "\n";

    } else {
        assert(multiThreadingType == MultiThreadingType::PTHREADS);

        printFunctionStartPThreads(_cache, hessInfo.size());
        _cache << "\n"
                "   for(i = 0; i < " << hessInfo.size() << "; ++i) {\n"
                "      args[i] = (ExecArgStruct*) malloc(sizeof(ExecArgStruct));\n"
                "      args[i]->func = p[i];\n"
                "      args[i]->in = inLocal;\n"
                "      args[i]->out[0] = &hess[offset[i]];\n"
                "      args[i]->atomicFun = " << langC .getArgumentAtomic() << ";\n"
                "   }\n"
                "\n";
        printFunctionEndPThreads(_cache, hessInfo.size());
    }

    _cache << "\n"
            "}\n";
    return _cache.str();
}

template<class Base>
void ModelCSourceGen<Base>::determineSecondOrderElements4Eval(std::vector<size_t>& evalRows,
                                                              std::vector<size_t>& evalCols) {
    /**
     * Atomic functions migth not have all the elements and thus there may 
     * be no symmetry. This will explore symmetry in order to provide the
     * second order elements requested by the user.
     */
    evalRows.reserve(_hessSparsity.rows.size());
    evalCols.reserve(_hessSparsity.cols.size());

    for (size_t e = 0; e < _hessSparsity.rows.size(); e++) {
        size_t i = _hessSparsity.rows[e];
        size_t j = _hessSparsity.cols[e];
        if (_hessSparsity.sparsity[i].find(j) == _hessSparsity.sparsity[i].end() &&
                _hessSparsity.sparsity[j].find(i) != _hessSparsity.sparsity[j].end()) {
            // only the symmetric value is available
            // (it can be caused by atomic functions which may only be providing a partial hessian)
            evalRows.push_back(j);
            evalCols.push_back(i);
        } else {
            evalRows.push_back(i);
            evalCols.push_back(j);
        }
    }
}

template<class Base>
void ModelCSourceGen<Base>::determineHessianSparsity() {
    if (_hessSparsity.sparsity.size() > 0) {
        return;
    }

    size_t m = _fun.Range();
    size_t n = _fun.Domain();

    /**
     * sparsity for the sum of the hessians of all equations
     */
    SparsitySetType r(n); // identity matrix
    for (size_t j = 0; j < n; j++)
        r[j].insert(j);
    SparsitySetType jac = _fun.ForSparseJac(n, r);

    SparsitySetType s(1);
    for (size_t i = 0; i < m; i++) {
        s[0].insert(i);
    }
    _hessSparsity.sparsity = _fun.RevSparseHes(n, s, false);
    //printSparsityPattern(_hessSparsity.sparsity, "hessian");

    if (_hessianByEquation || _reverseTwo) {
        /**
         * sparsity for the hessian of each equations
         */

        std::set<size_t> customVarsInHess;
        if (_custom_hess.defined) {
            customVarsInHess.insert(_custom_hess.row.begin(), _custom_hess.row.end());
            customVarsInHess.insert(_custom_hess.col.begin(), _custom_hess.col.end());

            r = SparsitySetType(n); //clear r
            for (size_t j : customVarsInHess) {
                r[j].insert(j);
            }
            jac = _fun.ForSparseJac(n, r);
        }

        /**
         * Coloring
         */
        const std::vector<Color> colors = colorByRow(customVarsInHess, jac);

        /**
         * For each individual equation
         */
        _hessSparsities.resize(m);
        for (size_t i = 0; i < m; i++) {
            _hessSparsities[i].sparsity.resize(n);
        }

        for (size_t c = 0; c < colors.size(); c++) {
            const Color& color = colors[c];

            // first-order
            r = SparsitySetType(n); //clear r
            for (size_t j : color.forbiddenRows) {
                r[j].insert(j);
            }
            _fun.ForSparseJac(n, r);

            // second-order
            s[0].clear();
            const std::set<size_t>& equations = color.rows;
            for (size_t i : equations) {
                s[0].insert(i);
            }

            SparsitySetType sparsityc = _fun.RevSparseHes(n, s, false);

            /**
             * Retrieve the individual hessians for each equation
             */
            const std::map<size_t, size_t>& var2Eq = color.column2Row;
            for (size_t j : color.forbiddenRows) { //used variables
                if (sparsityc[j].size() > 0) {
                    size_t i = var2Eq.at(j);
                    _hessSparsities[i].sparsity[j].insert(sparsityc[j].begin(),
                                                          sparsityc[j].end());
                }
            }

        }

        for (size_t i = 0; i < m; i++) {
            LocalSparsityInfo& hessSparsitiesi = _hessSparsities[i];

            if (!_custom_hess.defined) {
                generateSparsityIndexes(hessSparsitiesi.sparsity,
                                        hessSparsitiesi.rows, hessSparsitiesi.cols);

            } else {
                size_t nnz = _custom_hess.row.size();
                for (size_t e = 0; e < nnz; e++) {
                    size_t i1 = _custom_hess.row[e];
                    size_t i2 = _custom_hess.col[e];
                    if (hessSparsitiesi.sparsity[i1].find(i2) != hessSparsitiesi.sparsity[i1].end()) {
                        hessSparsitiesi.rows.push_back(i1);
                        hessSparsitiesi.cols.push_back(i2);
                    }
                }
            }
        }

    }

    if (!_custom_hess.defined) {
        generateSparsityIndexes(_hessSparsity.sparsity,
                                _hessSparsity.rows, _hessSparsity.cols);

    } else {
        _hessSparsity.rows = _custom_hess.row;
        _hessSparsity.cols = _custom_hess.col;
    }
}

template<class Base>
void ModelCSourceGen<Base>::generateHessianSparsitySource() {
    determineHessianSparsity();

    generateSparsity2DSource(_name + "_" + FUNCTION_HESSIAN_SPARSITY, _hessSparsity);
    _sources[_name + "_" + FUNCTION_HESSIAN_SPARSITY + ".c"] = _cache.str();
    _cache.str("");

    if (_hessianByEquation || _reverseTwo) {
        generateSparsity2DSource2(_name + "_" + FUNCTION_HESSIAN_SPARSITY2, _hessSparsities);
        _sources[_name + "_" + FUNCTION_HESSIAN_SPARSITY2 + ".c"] = _cache.str();
        _cache.str("");
    }
}

} // END cg namespace
} // END CppAD namespace

#endif