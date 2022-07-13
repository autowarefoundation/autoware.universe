#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_JAC_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_JAC_INCLUDED
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
void ModelCSourceGen<Base>::generateJacobianSource() {
    using std::vector;

    const std::string jobName = "Jacobian";

    startingJob("'" + jobName + "'", JobTimer::GRAPH);

    CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);

    vector<CGBase> indVars(_fun.Domain());
    handler.makeVariables(indVars);
    if (_x.size() > 0) {
        for (size_t i = 0; i < indVars.size(); i++) {
            indVars[i].setValue(_x[i]);
        }
    }

    size_t m = _fun.Range();
    size_t n = _fun.Domain();

    vector<CGBase> jac(n * m);
    if (_jacMode == JacobianADMode::Automatic) {
        jac = _fun.Jacobian(indVars);
    } else if (_jacMode == JacobianADMode::Forward) {
        JacobianFor(_fun, indVars, jac);
    } else {
        JacobianRev(_fun, indVars, jac);
    }

    finishedJob();

    LanguageC<Base> langC(_baseTypeName);
    langC.setMaxAssignmentsPerFunction(_maxAssignPerFunc, &_sources);
    langC.setMaxOperationsPerAssignment(_maxOperationsPerAssignment);
    langC.setParameterPrecision(_parameterPrecision);
    langC.setGenerateFunction(_name + "_" + FUNCTION_JACOBIAN);

    std::ostringstream code;
    std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator("jac"));

    handler.generateCode(code, langC, jac, *nameGen, _atomicFunctions, jobName);
}

template<class Base>
void ModelCSourceGen<Base>::generateSparseJacobianSource(MultiThreadingType multiThreadingType) {
    size_t m = _fun.Range();
    size_t n = _fun.Domain();

    /**
     * Determine the sparsity pattern
     */
    determineJacobianSparsity();

    bool forwardMode;

    if (_jacMode == JacobianADMode::Automatic) {
        if (_custom_jac.defined) {
            forwardMode = estimateBestJacobianADMode(_jacSparsity.rows, _jacSparsity.cols);
        } else {
            forwardMode = n <= m;
        }
    } else {
        forwardMode = _jacMode == JacobianADMode::Forward;
    }

    /**
     * call the appropriate method for source code generation
     */
    if (_sparseJacobianReusesOne && _forwardOne && forwardMode) {
        generateSparseJacobianForRevSource(true, multiThreadingType);
    } else if (_sparseJacobianReusesOne && _reverseOne && !forwardMode) {
        generateSparseJacobianForRevSource(false, multiThreadingType);
    } else {
        generateSparseJacobianSource(forwardMode);
    }
}

template<class Base>
void ModelCSourceGen<Base>::generateSparseJacobianSource(bool forward) {
    using std::vector;

    const std::string jobName = "sparse Jacobian";

    //size_t m = _fun.Range();
    size_t n = _fun.Domain();

    startingJob("'" + jobName + "'", JobTimer::GRAPH);

    CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);

    vector<CGBase> indVars(n);
    handler.makeVariables(indVars);
    if (_x.size() > 0) {
        for (size_t i = 0; i < n; i++) {
            indVars[i].setValue(_x[i]);
        }
    }

    vector<CGBase> jac(_jacSparsity.rows.size());
    if (_loopTapes.empty()) {
        //printSparsityPattern(_jacSparsity.sparsity, "jac sparsity");
        CppAD::sparse_jacobian_work work;
        if (forward) {
            _fun.SparseJacobianForward(indVars, _jacSparsity.sparsity, _jacSparsity.rows, _jacSparsity.cols, jac, work);
        } else {
            _fun.SparseJacobianReverse(indVars, _jacSparsity.sparsity, _jacSparsity.rows, _jacSparsity.cols, jac, work);
        }

    } else {
        jac = prepareSparseJacobianWithLoops(handler, indVars, forward);
    }

    finishedJob();

    LanguageC<Base> langC(_baseTypeName);
    langC.setMaxAssignmentsPerFunction(_maxAssignPerFunc, &_sources);
    langC.setMaxOperationsPerAssignment(_maxOperationsPerAssignment);
    langC.setParameterPrecision(_parameterPrecision);
    langC.setGenerateFunction(_name + "_" + FUNCTION_SPARSE_JACOBIAN);

    std::ostringstream code;
    std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator("jac"));

    handler.generateCode(code, langC, jac, *nameGen, _atomicFunctions, jobName);
}

template<class Base>
void ModelCSourceGen<Base>::generateSparseJacobianForRevSource(bool forward,
                                                               MultiThreadingType multiThreadingType) {
    //size_t m = _fun.Range();
    //size_t n = _fun.Domain();
    using namespace std;

    std::map<size_t, CompressedVectorInfo> jacInfo;
    string functionRevFor, revForSuffix;
    if (forward) {
        // jacInfo[var].index{equations}
        for (size_t e = 0; e < _jacSparsity.rows.size(); e++) {
            jacInfo[_jacSparsity.cols[e]].indexes.push_back(_jacSparsity.rows[e]);
        }
        for (auto& it : jacInfo) {
            size_t col = it.first;
            it.second.locations = determineOrderByCol(col, it.second.indexes, _jacSparsity.rows, _jacSparsity.cols);
        }
        _cache.str("");
        _cache << _name << "_" << FUNCTION_SPARSE_FORWARD_ONE;
        functionRevFor = _cache.str();
        revForSuffix = "indep";
    } else {
        // jacInfo[equation].index{vars}
        for (size_t e = 0; e < _jacSparsity.rows.size(); e++) {
            jacInfo[_jacSparsity.rows[e]].indexes.push_back(_jacSparsity.cols[e]);
        }
        for (auto& it : jacInfo) {
            size_t row = it.first;
            it.second.locations = determineOrderByRow(row, it.second.indexes, _jacSparsity.rows, _jacSparsity.cols);
        }
        _cache.str("");
        _cache << _name << "_" << FUNCTION_SPARSE_REVERSE_ONE;
        functionRevFor = _cache.str();
        revForSuffix = "dep";
    }


    /**
     * determine to which functions we can provide the jacobian row/column
     * directly without needing a temporary array (compressed)
     */
    for (auto& it : jacInfo) {
        const std::vector<size_t>& els = it.second.indexes;
        const std::vector<set<size_t> >& location = it.second.locations;
        CPPADCG_ASSERT_UNKNOWN(els.size() == location.size());
        CPPADCG_ASSERT_UNKNOWN(els.size() > 0);

        bool passed = true;
        size_t jacArrayStart = *location[0].begin();
        for (size_t e = 0; e < els.size(); e++) {
            if (location[e].size() > 1) {
                passed = false; // too many elements
                break;
            }
            if (*location[e].begin() != jacArrayStart + e) {
                passed = false; // wrong order
                break;
            }
        }
        it.second.ordered = passed;
    }

    size_t maxCompressedSize = 0;
    map<size_t, bool>::const_iterator itOrd;
    map<size_t, std::vector<size_t> >::const_iterator it;
    for (const auto& it : jacInfo) {
        if (it.second.indexes.size() > maxCompressedSize && !it.second.ordered)
            maxCompressedSize = it.second.indexes.size();
    }

    if (!_loopTapes.empty()) {
        /**
         * with loops
         */
        if (forward) {
            generateSparseJacobianWithLoopsSourceFromForRev(jacInfo, maxCompressedSize,
                                                            FUNCTION_SPARSE_FORWARD_ONE, "indep", "jcol",
                                                            _nonLoopFor1Elements, _loopFor1Groups,
                                                            generateFunctionNameLoopFor1);
        } else {
            generateSparseJacobianWithLoopsSourceFromForRev(jacInfo, maxCompressedSize,
                                                            FUNCTION_SPARSE_REVERSE_ONE, "dep", "jrow",
                                                            _nonLoopRev1Elements, _loopRev1Groups,
                                                            generateFunctionNameLoopRev1);
        }
        return;
    }

    _cache.str("");
    _cache << _name << "_" << FUNCTION_SPARSE_JACOBIAN;
    string functionName(_cache.str());

    if(!_multiThreading || multiThreadingType == MultiThreadingType::NONE) {
        _sources[functionName + ".c"] = generateSparseJacobianForRevSingleThreadSource(functionName, jacInfo, maxCompressedSize, functionRevFor, revForSuffix, forward);
    } else {
        _sources[functionName + ".c"] = generateSparseJacobianForRevMultiThreadSource(functionName, jacInfo, maxCompressedSize, functionRevFor, revForSuffix, forward, multiThreadingType);
    }

    _cache.str("");
}

template<class Base>
std::string ModelCSourceGen<Base>::generateSparseJacobianForRevSingleThreadSource(const std::string& functionName,
                                                                                  std::map<size_t, CompressedVectorInfo> jacInfo,
                                                                                  size_t maxCompressedSize,
                                                                                  const std::string& functionRevFor,
                                                                                  const std::string& revForSuffix,
                                                                                  bool forward) {
    
    LanguageC<Base> langC(_baseTypeName);
    std::string argsDcl = langC.generateDefaultFunctionArgumentsDcl();
    std::vector<std::string> argsDcl2 = langC.generateDefaultFunctionArgumentsDcl2();

    _cache.str("");
    _cache << "#include <stdlib.h>\n"
            "\n"
           << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n\n";
    generateFunctionDeclarationSource(_cache, functionRevFor, revForSuffix, jacInfo, argsDcl);
    _cache << "\n";
    LanguageC<Base>::printFunctionDeclaration(_cache, "void", functionName, argsDcl2);
    _cache << " {\n"
              "   " << _baseTypeName << " const * inLocal[2];\n"
              "   " << _baseTypeName << " inLocal1 = 1;\n"
              "   " << _baseTypeName << " * outLocal[1];\n"
              "   " << _baseTypeName << " compressed[" << maxCompressedSize << "];\n"
              "   " << _baseTypeName << " * jac = out[0];\n"
              "\n"
              "   inLocal[0] = in[0];\n"
              "   inLocal[1] = &inLocal1;\n"
              "   outLocal[0] = compressed;\n";

    langC.setArgumentIn("inLocal");
    langC.setArgumentOut("outLocal");
    std::string argsLocal = langC.generateDefaultFunctionArguments();

    bool previousCompressed = true;
    for (const auto& it : jacInfo) {
        size_t index = it.first;
        const std::vector<size_t>& els = it.second.indexes;
        const std::vector<std::set<size_t> >& location = it.second.locations;
        CPPADCG_ASSERT_UNKNOWN(els.size() == location.size());

        _cache << "\n";
        bool compressed = !it.second.ordered;
        if (!compressed) {
            _cache << "   outLocal[0] = &jac[" << *location[0].begin() << "];\n";
        } else if (!previousCompressed) {
            _cache << "   outLocal[0] = compressed;\n";
        }
        _cache << "   " << functionRevFor << "_" << revForSuffix << index << "(" << argsLocal << ");\n";
        if (compressed) {
            for (size_t e = 0; e < els.size(); e++) {
                _cache << "   ";
                for (size_t itl : location[e]) {
                    _cache << "jac[" << (itl) << "] = ";
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
std::string ModelCSourceGen<Base>::generateSparseJacobianForRevMultiThreadSource(const std::string& functionName,
                                                                                 std::map<size_t, CompressedVectorInfo> jacInfo,
                                                                                 size_t maxCompressedSize,
                                                                                 const std::string& functionRevFor,
                                                                                 const std::string& revForSuffix,
                                                                                 bool forward,
                                                                                 MultiThreadingType multiThreadingType) {
    LanguageC<Base> langC(_baseTypeName);
    std::string argsDcl = langC.generateDefaultFunctionArgumentsDcl();
    std::vector<std::string> argsDcl2 = langC.generateDefaultFunctionArgumentsDcl2();

    _cache.str("");
    _cache << "#include <stdlib.h>\n"
            "\n"
           << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n\n";
    generateFunctionDeclarationSource(_cache, functionRevFor, revForSuffix, jacInfo, argsDcl);

    langC.setArgumentIn("inLocal");
    langC.setArgumentOut("outLocal");
    std::string argsLocal = langC.generateDefaultFunctionArguments();

    /**
     * Create independent functions for each row/column of the Jacobian
     */
    for (const auto& it : jacInfo) {
        size_t index = it.first;
        const std::vector<size_t>& els = it.second.indexes;
        const std::vector<std::set<size_t> >& location = it.second.locations;
        CPPADCG_ASSERT_UNKNOWN(els.size() == location.size());

        bool compressed = !it.second.ordered;
        if (!compressed) {
            continue;
        }

        std::string functionNameWrap = functionRevFor + "_" + revForSuffix + std::to_string(index) + "_wrap";
        LanguageC<Base>::printFunctionDeclaration(_cache, "void", functionNameWrap, argsDcl2);
        _cache << " {\n"
                "   " << _baseTypeName << " const * inLocal[2];\n"
                        "   " << _baseTypeName << " inLocal1 = 1;\n"
                        "   " << _baseTypeName << " * outLocal[1];\n"
                        "   " << _baseTypeName << " compressed[" << it.second.indexes.size() << "];\n"
                        "   " << _baseTypeName << " * jac = out[0];\n"
                        "\n"
                        "   inLocal[0] = in[0];\n"
                        "   inLocal[1] = &inLocal1;\n"
                        "   outLocal[0] = compressed;\n";

        _cache << "   " << functionRevFor << "_" << revForSuffix << index << "(" << argsLocal << ");\n";
        for (size_t e = 0; e < els.size(); e++) {
            _cache << "   ";
            for (size_t itl : location[e]) {
                _cache << "jac[" << (itl) << "] = ";
            }
            _cache << "compressed[" << e << "];\n";
        }
        _cache << "}\n";
    }

    _cache << "\n"
            "typedef void (*cppadcg_function_type) (" << argsDcl << ");\n";

    /**
     * PThreads pool needs a function with a void pointer argument
     */
    if(multiThreadingType == MultiThreadingType::OPENMP) {
        _cache << "\n";
        printFileStartOpenMP(_cache);
        _cache << "\n";

    } else {
        assert(multiThreadingType == MultiThreadingType::PTHREADS);

        printFileStartPThreads(_cache, _baseTypeName);
    }

    /**
     * Jacobian function
     */
    _cache << "\n"
            "void " << functionName << "(" << argsDcl << ") {\n"
            "   static const cppadcg_function_type p[" << jacInfo.size() << "] = {";
    for (const auto& it : jacInfo) {
        size_t index = it.first;
        if (index != jacInfo.begin()->first) _cache << ", ";
        if (it.second.ordered) {
            _cache << functionRevFor << "_" << revForSuffix << index;
        } else {
            _cache << functionRevFor << "_" << revForSuffix << index << "_wrap";
        }
    }
    _cache << "};\n"
            "   static const long offset["<< jacInfo.size() <<"] = {";
    for (const auto& it : jacInfo) {
        if (it.first != jacInfo.begin()->first) _cache << ", ";
        if (it.second.ordered) {
            _cache << *it.second.locations[0].begin();
        } else {
            _cache << "0";
        }
    }
    _cache << "};\n"
            "   " << _baseTypeName << " inLocal1 = 1;\n"
            "   " << _baseTypeName << " const * inLocal[2] = {in[0], &inLocal1};\n"
            "   " << _baseTypeName << " * outLocal[1];\n"
            "   " << _baseTypeName << " * jac = out[0];\n"
            "   long i;\n"
            "\n";

    if(multiThreadingType == MultiThreadingType::OPENMP) {
        printFunctionStartOpenMP(_cache, jacInfo.size());
        _cache << "\n";
        printLoopStartOpenMP(_cache, jacInfo.size());
        _cache << "      outLocal[0] = &jac[offset[i]];\n"
                "      (*p[i])(" << argsLocal << ");\n";
        printLoopEndOpenMP(_cache, jacInfo.size());
        _cache << "\n";

    } else {
        assert(multiThreadingType == MultiThreadingType::PTHREADS);

        printFunctionStartPThreads(_cache, jacInfo.size());
        _cache << "\n"
                "   for(i = 0; i < " << jacInfo.size() << "; ++i) {\n"
                "      args[i] = (ExecArgStruct*) malloc(sizeof(ExecArgStruct));\n"
                "      args[i]->func = p[i];\n"
                "      args[i]->in = inLocal;\n"
                "      args[i]->out[0] = &jac[offset[i]];\n"
                "      args[i]->atomicFun = " << langC.getArgumentAtomic() << ";\n"
                "   }\n"
                "\n";
        printFunctionEndPThreads(_cache, jacInfo.size());
    }

    _cache << "\n"
            "}\n";

    return _cache.str();
}

template<class Base>
void ModelCSourceGen<Base>::determineJacobianSparsity() {
    if (_jacSparsity.sparsity.size() > 0) {
        return;
    }

    /**
     * Determine the sparsity pattern
     */
    _jacSparsity.sparsity = jacobianSparsitySet<SparsitySetType, CGBase> (_fun);

    if (!_custom_jac.defined) {
        generateSparsityIndexes(_jacSparsity.sparsity, _jacSparsity.rows, _jacSparsity.cols);

    } else {
        _jacSparsity.rows = _custom_jac.row;
        _jacSparsity.cols = _custom_jac.col;
    }
}

template<class Base>
void ModelCSourceGen<Base>::generateJacobianSparsitySource() {
    determineJacobianSparsity();

    generateSparsity2DSource(_name + "_" + FUNCTION_JACOBIAN_SPARSITY, _jacSparsity);
    _sources[_name + "_" + FUNCTION_JACOBIAN_SPARSITY + ".c"] = _cache.str();
    _cache.str("");
}

} // END cg namespace
} // END CppAD namespace

#endif