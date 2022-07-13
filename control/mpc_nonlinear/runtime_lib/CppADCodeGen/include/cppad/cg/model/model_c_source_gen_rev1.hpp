#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_REV1_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_REV1_INCLUDED
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
void ModelCSourceGen<Base>::generateSparseReverseOneSources() {

    determineJacobianSparsity();

    // elements[equation]{vars}
    std::map<size_t, std::vector<size_t> > elements;
    for (size_t e = 0; e < _jacSparsity.rows.size(); e++) {
        elements[_jacSparsity.rows[e]].push_back(_jacSparsity.cols[e]);
    }

    if (!_loopTapes.empty()) {
        /**
         * with loops
         */
        prepareSparseReverseOneWithLoops(elements);
        return;
    }

    /**
     * Generate one function for each dependent variable
     */
    startingJob("'model (reverse one)'", JobTimer::SOURCE_GENERATION);

    if (isAtomicsUsed()) {
        generateSparseReverseOneSourcesWithAtomics(elements);
    } else {
        generateSparseReverseOneSourcesNoAtomics(elements);
    }

    finishedJob();

    _cache.str("");

    generateGlobalDirectionalFunctionSource(FUNCTION_SPARSE_REVERSE_ONE,
                                            "dep",
                                            FUNCTION_REVERSE_ONE_SPARSITY,
                                            elements);
}

template<class Base>
void ModelCSourceGen<Base>::generateSparseReverseOneSourcesWithAtomics(const std::map<size_t, std::vector<size_t> >& elements) {
    using std::vector;

    /**
     * Generate one function for each dependent variable
     */
    size_t m = _fun.Range();
    size_t n = _fun.Domain();

    vector<CGBase> w(m);

    /**
     * Generate one function for each dependent variable
     */
    const std::string jobName = "model (reverse one)";
    startingJob("'" + jobName + "'", JobTimer::SOURCE_GENERATION);

    for (const auto& it : elements) {
        size_t i = it.first;
        const std::vector<size_t>& cols = it.second;

        _cache.str("");
        _cache << "model (reverse one, dep " << i << ")";
        const std::string subJobName = _cache.str();

        startingJob("'" + subJobName + "'", JobTimer::GRAPH);

        CodeHandler<Base> handler;
        handler.setJobTimer(_jobTimer);

        vector<CGBase> indVars(_fun.Domain());
        handler.makeVariables(indVars);
        if (_x.size() > 0) {
            for (size_t i = 0; i < n; i++) {
                indVars[i].setValue(_x[i]);
            }
        }

        CGBase py;
        handler.makeVariable(py);
        if (_x.size() > 0) {
            py.setValue(Base(1.0));
        }

        // TODO: consider caching the zero order coefficients somehow between calls
        _fun.Forward(0, indVars);

        w[i] = py;
        vector<CGBase> dw = _fun.Reverse(1, w);
        CPPADCG_ASSERT_UNKNOWN(dw.size() == n);
        w[i] = Base(0);

        vector<CGBase> dwCustom;
        for (size_t it2 : cols) {
            dwCustom.push_back(dw[it2]);
        }

        finishedJob();

        LanguageC<Base> langC(_baseTypeName);
        langC.setMaxAssignmentsPerFunction(_maxAssignPerFunc, &_sources);
        langC.setMaxOperationsPerAssignment(_maxOperationsPerAssignment);
        langC.setParameterPrecision(_parameterPrecision);
        _cache.str("");
        _cache << _name << "_" << FUNCTION_SPARSE_REVERSE_ONE << "_dep" << i;
        langC.setGenerateFunction(_cache.str());

        std::ostringstream code;
        std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator("dw"));
        LangCDefaultHessianVarNameGenerator<Base> nameGenHess(nameGen.get(), "py", n);

        handler.generateCode(code, langC, dwCustom, nameGenHess, _atomicFunctions, subJobName);
    }
}

template<class Base>
void ModelCSourceGen<Base>::generateSparseReverseOneSourcesNoAtomics(const std::map<size_t, std::vector<size_t> >& elements) {
    using std::vector;

    /**
     * Jacobian
     */
    size_t m = _fun.Range();
    size_t n = _fun.Domain();

    CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);

    vector<CGBase> x(n);
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

    vector<CGBase> jacFlat(_jacSparsity.rows.size());

    CppAD::sparse_jacobian_work work; // temporary structure for CPPAD
    _fun.SparseJacobianReverse(x, _jacSparsity.sparsity, _jacSparsity.rows, _jacSparsity.cols, jacFlat, work);

    /**
     * organize results
     */
    std::map<size_t, vector<CGBase> > jac; // by row
    std::vector<std::map<size_t, size_t> > positions(m); // by row

    for (const auto& it : elements) {
        size_t i = it.first;
        const std::vector<size_t>& row = it.second;

        jac[i].resize(row.size());
        std::map<size_t, size_t>& pos = positions[i];

        for (size_t e = 0; e < row.size(); e++) {
            size_t j = row[e];
            pos[j] = e;
        }
    }

    for (size_t el = 0; el < _jacSparsity.rows.size(); el++) {
        size_t i = _jacSparsity.rows[el];
        size_t j = _jacSparsity.cols[el];
        size_t e = positions[i].at(j);

        vector<CGBase>& row = jac[i];
        row[e] = jacFlat[el] * py;
    }

    /**
     * Create source for each equation/row
     */
    typename std::map<size_t, vector<CGBase> >::iterator itI;
    for (itI = jac.begin(); itI != jac.end(); ++itI) {
        size_t i = itI->first;
        vector<CGBase>& dwCustom = itI->second;

        _cache.str("");
        _cache << "model (reverse one, dep " << i << ")";
        const std::string subJobName = _cache.str();

        LanguageC<Base> langC(_baseTypeName);
        langC.setMaxAssignmentsPerFunction(_maxAssignPerFunc, &_sources);
        langC.setMaxOperationsPerAssignment(_maxOperationsPerAssignment);
        langC.setParameterPrecision(_parameterPrecision);
        _cache.str("");
        _cache << _name << "_" << FUNCTION_SPARSE_REVERSE_ONE << "_dep" << i;
        langC.setGenerateFunction(_cache.str());

        std::ostringstream code;
        std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator("dw"));
        LangCDefaultHessianVarNameGenerator<Base> nameGenHess(nameGen.get(), "py", n);

        handler.generateCode(code, langC, dwCustom, nameGenHess, _atomicFunctions, subJobName);
    }
}

template<class Base>
void ModelCSourceGen<Base>::generateReverseOneSources() {
    size_t m = _fun.Range();
    size_t n = _fun.Domain();

    _cache.str("");
    _cache << _name << "_" << FUNCTION_REVERSE_ONE;
    std::string model_function(_cache.str());
    _cache.str("");

    LanguageC<Base> langC(_baseTypeName);
    std::string argsDcl = langC.generateDefaultFunctionArgumentsDcl();
    std::string args = langC.generateDefaultFunctionArguments();

    _cache << "#include <stdlib.h>\n"
            << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n"
            "\n"
            "int " << _name << "_" << FUNCTION_SPARSE_REVERSE_ONE << "(unsigned long pos, " << argsDcl << ");\n"
            "void " << _name << "_" << FUNCTION_REVERSE_ONE_SPARSITY << "(unsigned long pos, unsigned long const** elements, unsigned long* nnz);\n"
            "\n";
    LanguageC<Base>::printFunctionDeclaration(_cache, "int", model_function, {_baseTypeName + " const x[]",
                                                                              _baseTypeName + " const ty[]",
                                                                              _baseTypeName + " px[]",
                                                                              _baseTypeName + " const py[]",
                                                                              langC.generateArgumentAtomicDcl()});
    _cache << " {\n"
            "   unsigned long ei, ePos, i, j, nnz, nnzMax;\n"
            "   unsigned long const* pos;\n"
            "   unsigned long* pyPos;\n"
            "   unsigned long* pyPosTmp;\n"
            "   unsigned long nnzPy;\n"
            "   " << _baseTypeName << " const * in[2];\n"
            "   " << _baseTypeName << "* out[1];\n"
            "   " << _baseTypeName << "* compressed;\n"
            "   int ret;\n"
            "\n"
            "   pyPos = 0;\n"
            "   nnzPy = 0;\n"
            "   nnzMax = 0;\n"
            "   for (i = 0; i < " << m << "; i++) {\n"
            "      if (py[i] != 0.0) {\n"
            "         " << _name << "_" << FUNCTION_REVERSE_ONE_SPARSITY << "(i, &pos, &nnz);\n"
            "         if (nnz > nnzMax)\n"
            "            nnzMax = nnz;\n"
            "         else if (nnz == 0)\n"
            "            continue;\n"
            "         nnzPy++;\n"
            "         pyPosTmp = (unsigned long*) realloc(pyPos, nnzPy * sizeof(unsigned long));\n"
            "         if (pyPosTmp != NULL) {\n"
            "            pyPos = pyPosTmp;\n"
            "         } else {\n"
            "            free(pyPos);\n"
            "            return -1; // failure to allocate memory\n"
            "         }\n"
            "         pyPos[nnzPy - 1] = i;\n"
            "      }\n"
            "   }\n"
            "   for (j = 0; j < " << n << "; j++) {\n"
            "      px[j] = 0;\n"
            "   }\n"
            "\n"
            "   if (nnzPy == 0) {\n"
            "      free(pyPos);\n"
            "      return 0; //nothing to do\n"
            "   }\n"
            "\n"
            "   compressed = (" << _baseTypeName << "*) malloc(nnzMax * sizeof(" << _baseTypeName << "));\n"
            "\n"
            "   for (ei = 0; ei < nnzPy; ei++) {\n"
            "      i = pyPos[ei];\n"
            "      " << _name << "_" << FUNCTION_REVERSE_ONE_SPARSITY << "(i, &pos, &nnz);\n"
            "\n"
            "      in[0] = x;\n"
            "      in[1] = &py[i];\n"
            "      out[0] = compressed;\n";
    if (!_loopTapes.empty()) {
        _cache << "      for(ePos = 0; ePos < nnz; ePos++)\n"
                "         compressed[ePos] = 0;\n"
                "\n";
    }
    _cache << "      ret = " << _name << "_" << FUNCTION_SPARSE_REVERSE_ONE << "(i, " << args << ");\n"
            "\n"
            "      if (ret != 0) {\n"
            "         free(compressed);\n"
            "         free(pyPos);\n"
            "         return ret;\n"
            "      }\n"
            "\n"
            "      for (ePos = 0; ePos < nnz; ePos++) {\n"
            "         px[pos[ePos]] += compressed[ePos];\n"
            "      }\n"
            "\n"
            "   }\n"
            "   free(compressed);\n"
            "   free(pyPos);\n"
            "   return 0;\n"
            "}\n";
    _sources[model_function + ".c"] = _cache.str();
    _cache.str("");
}

} // END cg namespace
} // END CppAD namespace

#endif