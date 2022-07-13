#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_IMPL_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_IMPL_INCLUDED
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

#include <typeinfo>

namespace CppAD {
namespace cg {

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_FORWAD_ZERO = "forward_zero";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_JACOBIAN = "jacobian";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_HESSIAN = "hessian";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_FORWARD_ONE = "forward_one";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_REVERSE_ONE = "reverse_one";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_REVERSE_TWO = "reverse_two";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_SPARSE_JACOBIAN = "sparse_jacobian";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_SPARSE_HESSIAN = "sparse_hessian";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_JACOBIAN_SPARSITY = "jacobian_sparsity";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_HESSIAN_SPARSITY = "hessian_sparsity";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_HESSIAN_SPARSITY2 = "hessian_sparsity2";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_SPARSE_FORWARD_ONE = "sparse_forward_one";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_SPARSE_REVERSE_ONE = "sparse_reverse_one";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_SPARSE_REVERSE_TWO = "sparse_reverse_two";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_FORWARD_ONE_SPARSITY = "forward_one_sparsity";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_REVERSE_ONE_SPARSITY = "reverse_one_sparsity";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_REVERSE_TWO_SPARSITY = "sparse_reverse_two_sparsity";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_INFO = "info";

template<class Base>
const std::string ModelCSourceGen<Base>::FUNCTION_ATOMIC_FUNC_NAMES = "atomic_functions";

template<class Base>
const std::string ModelCSourceGen<Base>::CONST = "const";

template<class Base>
VariableNameGenerator<Base>* ModelCSourceGen<Base>::createVariableNameGenerator(const std::string& depName,
                                                                                const std::string& indepName,
                                                                                const std::string& tmpName,
                                                                                const std::string& tmpArrayName) {
    return new LangCDefaultVariableNameGenerator<Base> (depName, indepName, tmpName, tmpArrayName);
}

template<class Base>
const std::map<std::string, std::string>& ModelCSourceGen<Base>::getSources(MultiThreadingType multiThreadingType,
                                                                            JobTimer* timer) {
    if (_sources.empty()) {
        generateSources(multiThreadingType, timer);
    }
    return _sources;
}

template<class Base>
void ModelCSourceGen<Base>::generateSources(MultiThreadingType multiThreadingType,
                                            JobTimer* timer) {
    _jobTimer = timer;

    generateLoops();

    startingJob("'" + _name + "'", JobTimer::SOURCE_FOR_MODEL);

    if (_zero) {
        generateZeroSource();
        _zeroEvaluated = true;
    }

    if (_jacobian) {
        generateJacobianSource();
    }

    if (_hessian) {
        generateHessianSource();
    }

    if (_forwardOne) {
        generateSparseForwardOneSources();
        generateForwardOneSources();
    }

    if (_reverseOne) {
        generateSparseReverseOneSources();
        generateReverseOneSources();
    }

    if (_reverseTwo) {
        generateSparseReverseTwoSources();
        generateReverseTwoSources();
    }

    if (_sparseJacobian) {
        generateSparseJacobianSource(multiThreadingType);
    }

    if (_sparseHessian) {
        generateSparseHessianSource(multiThreadingType);
    }

    if (_sparseJacobian || _forwardOne || _reverseOne) {
        generateJacobianSparsitySource();
    }

    if (_sparseHessian || _reverseTwo) {
        generateHessianSparsitySource();
    }

    generateInfoSource();

    generateAtomicFuncNames();

    finishedJob();
}

template<class Base>
void ModelCSourceGen<Base>::generateLoops() {
    if (_relatedDepCandidates.empty()) {
        return; //nothing to do
    }

    startingJob("", JobTimer::LOOP_DETECTION);

    CodeHandler<Base> handler;
    handler.setJobTimer(_jobTimer);

    std::vector<CGBase> xx(_fun.Domain());
    handler.makeVariables(xx);
    if (_x.size() > 0) {
        for (size_t i = 0; i < xx.size(); i++) {
            xx[i].setValue(_x[i]);
        }
    }

    std::vector<CGBase> yy = _fun.Forward(0, xx);

    DependentPatternMatcher<Base> matcher(_relatedDepCandidates, yy, xx);
    matcher.generateTapes(_funNoLoops, _loopTapes);

    finishedJob();
    if (_jobTimer != nullptr && _jobTimer->isVerbose()) {
        std::cout << " equation patterns: " << matcher.getEquationPatterns().size() <<
                "  loops: " << matcher.getLoops().size() << std::endl;
    }
}

template<class Base>
void ModelCSourceGen<Base>::generateInfoSource() {
    const char* localBaseName = typeid (Base).name();

    std::string funcName = _name + "_" + FUNCTION_INFO;

    std::unique_ptr<VariableNameGenerator<Base> > nameGen(createVariableNameGenerator());

    _cache.str("");
    LanguageC<Base>::printFunctionDeclaration(_cache, "void", funcName, {"const char** baseName",
                                                                         "unsigned long* m",
                                                                         "unsigned long* n",
                                                                         "unsigned int* indCount",
                                                                         "unsigned int* depCount"});
    _cache << " {\n"
            "   *baseName = \"" << _baseTypeName << "  " << localBaseName << "\";\n"
            "   *m = " << _fun.Range() << ";\n"
            "   *n = " << _fun.Domain() << ";\n"
            "   *depCount = " << nameGen->getDependent().size() << "; // number of dependent array variables\n"
            "   *indCount = " << nameGen->getIndependent().size() << "; // number of independent array variables\n"
            "}\n\n";

    _sources[funcName + ".c"] = _cache.str();
}

template<class Base>
void ModelCSourceGen<Base>::generateAtomicFuncNames() {
    std::string funcName = _name + "_" + FUNCTION_ATOMIC_FUNC_NAMES;
    size_t n = _atomicFunctions.size();
    _cache.str("");
    LanguageC<Base>::printFunctionDeclaration(_cache, "void", funcName, {"const char*** names",
                                                                         "unsigned long* n"});
    _cache << " {\n"
            "   static const char* atomic[" << n << "] = {";
    for (size_t i = 0; i < n; i++) {
        if (i > 0) _cache << ", ";
        _cache << "\"" << _atomicFunctions[i] << "\"";
    }
    _cache << "};\n"
            "   *names = atomic;\n"
            "   *n = " << n << ";\n"
            "}\n\n";

    _sources[funcName + ".c"] = _cache.str();
}

template<class Base>
bool ModelCSourceGen<Base>::isAtomicsUsed() {
    if (_zeroEvaluated) {
        return _atomicFunctions.size() > 0;
    } else {
        return !getAtomicsInfo().empty();
    }
}

template<class Base>
const std::map<size_t, AtomicUseInfo<Base> >& ModelCSourceGen<Base>::getAtomicsInfo() {
    if (_atomicsInfo == nullptr) {
        AtomicDependencyLocator<Base> adl(_fun);
        _atomicsInfo = new std::map<size_t, AtomicUseInfo<Base> >(adl.findAtomicsUsage());
    }
    return *_atomicsInfo;
}

template<class Base>
std::vector<typename ModelCSourceGen<Base>::Color> ModelCSourceGen<Base>::colorByRow(const std::set<size_t>& columns,
                                                                                     const SparsitySetType& sparsity) {
    std::vector<Color> colors(sparsity.size()); // reserve the maximum size to avoid reallocating more space later

    /**
     * try not match the columns of each row to a color which did not have
     * those columns yet
     */
    size_t c_used = 0;
    for (size_t i = 0; i < sparsity.size(); i++) {
        const std::set<size_t>& row = sparsity[i];
        if (row.size() == 0) {
            continue; //nothing to do
        }

        // consider only the columns present in the sparsity pattern
        std::set<size_t> rowReduced;
        if (_custom_hess.defined) {
            for (size_t j : row) {
                if (columns.find(j) != columns.end())
                    rowReduced.insert(j);
            }
        } else {
            rowReduced = row;
        }

        bool newColor = true;
        size_t colori;
        for (size_t c = 0; c < c_used; c++) {
            std::set<size_t>& forbidden_c = colors[c].forbiddenRows;
            if (!intersects(forbidden_c, rowReduced)) {
                // no intersection
                colori = c;
                newColor = false;
                forbidden_c.insert(rowReduced.begin(), rowReduced.end());
                break;
            }
        }

        if (newColor) {
            colori = c_used;
            colors[c_used].forbiddenRows = rowReduced;
            c_used++;
        }

        colors[colori].rows.insert(i);

        for (size_t j : rowReduced) {
            colors[colori].column2Row[j] = i;
            colors[colori].row2Columns[i].insert(j);
        }
    }

    colors.resize(c_used); //reduce size
    return colors;
}

template<class Base>
void ModelCSourceGen<Base>::generateGlobalDirectionalFunctionSource(const std::string& function,
                                                                    const std::string& suffix,
                                                                    const std::string& function_sparsity,
                                                                    const std::map<size_t, std::vector<size_t> >& elements) {
    /**
     * The function that matches each equation to a directional derivative function
     */
    LanguageC<Base> langC(_baseTypeName);
    std::string argsDcl = langC.generateDefaultFunctionArgumentsDcl();
    std::vector<std::string> argsDcl2 = langC.generateDefaultFunctionArgumentsDcl2();
    std::string args = langC.generateDefaultFunctionArguments();

    _cache.str("");
    _cache << _name << "_" << function;
    std::string model_function = _cache.str();
    _cache.str("");

    _cache << LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION << "\n\n";
    generateFunctionDeclarationSource(_cache, model_function, suffix, elements, argsDcl);
    _cache << "\n";
    LanguageC<Base>::printFunctionDeclaration(_cache, "int", model_function, {"unsigned long pos"}, argsDcl2);
    _cache << " {\n"
            "   switch(pos) {\n";
    for (const auto& it : elements) {
        // the size of each sparsity row
        _cache << "      case " << it.first << ":\n"
                "         " << model_function << "_" << suffix << it.first << "(" << args << ");\n"
                "         return 0; // done\n";
    }
    _cache << "      default:\n"
            "         return 1; // error\n"
            "   };\n";

    _cache << "}\n";
    _sources[model_function + ".c"] = _cache.str();
    _cache.str("");

    /**
     * Sparsity
     */
    generateSparsity1DSource2(_name + "_" + function_sparsity, elements);
    _sources[_name + "_" + function_sparsity + ".c"] = _cache.str();
    _cache.str("");
}

template<class Base>
template<class T>
void ModelCSourceGen<Base>::generateFunctionDeclarationSource(std::ostringstream& cache,
                                                              const std::string& model_function,
                                                              const std::string& suffix,
                                                              const std::map<size_t, T>& elements,
                                                              const std::string& argsDcl) {
    for (const auto& it : elements) {
        size_t pos = it.first;
        cache << "void " << model_function << "_" << suffix << pos << "(" << argsDcl << ");\n";
    }
}

template<class Base>
void ModelCSourceGen<Base>::generateSparsity1DSource(const std::string& function,
                                                     const std::vector<size_t>& sparsity) {
    LanguageC<Base>::printFunctionDeclaration(_cache, "void", function, {"unsigned long const** sparsity",
                                                                         "unsigned long* nnz"});
    _cache << " {\n";

    // the size of each sparsity row
    _cache << "   ";
    LanguageC<Base>::printStaticIndexArray(_cache, "nonzeros", sparsity);

    _cache << "   *sparsity = nonzeros;\n"
            "   *nnz = " << sparsity.size() << ";\n"
            "}\n";
}

template<class Base>
void ModelCSourceGen<Base>::generateSparsity2DSource(const std::string& function,
                                                     const LocalSparsityInfo& sparsity) {
    const std::vector<size_t>& rows = sparsity.rows;
    const std::vector<size_t>& cols = sparsity.cols;

    CPPADCG_ASSERT_UNKNOWN(rows.size() == cols.size());

    LanguageC<Base>::printFunctionDeclaration(_cache, "void", function, {"unsigned long const** row",
                                                                         "unsigned long const** col",
                                                                         "unsigned long* nnz"});
    _cache << " {\n";

    // the size of each sparsity row
    _cache << "   ";
    LanguageC<Base>::printStaticIndexArray(_cache, "rows", rows);

    _cache << "   ";
    LanguageC<Base>::printStaticIndexArray(_cache, "cols", cols);

    _cache << "   *row = rows;\n"
            "   *col = cols;\n"
            "   *nnz = " << rows.size() << ";\n"
            "}\n";
}

template<class Base>
void ModelCSourceGen<Base>::generateSparsity2DSource2(const std::string& function,
                                                      const std::vector<LocalSparsityInfo>& sparsities) {
    LanguageC<Base>::printFunctionDeclaration(_cache, "void", function, {"unsigned long i",
                                                                         "unsigned long const** row",
                                                                         "unsigned long const** col",
                                                                         "unsigned long* nnz"});
    _cache << " {\n";

    std::ostringstream os;

    std::vector<size_t> nnzs(sparsities.size());

    long long int maxNnzIndex = -1;

    for (size_t i = 0; i < sparsities.size(); i++) {
        const std::vector<size_t>& rows = sparsities[i].rows;
        const std::vector<size_t>& cols = sparsities[i].cols;
        CPPADCG_ASSERT_UNKNOWN(rows.size() == cols.size());

        nnzs[i] = rows.size();

        if (!rows.empty()) {
            os.str("");
            os << "rows" << i;
            _cache << "   ";
            LanguageC<Base>::printStaticIndexArray(_cache, os.str(), rows);

            os.str("");
            os << "cols" << i;
            _cache << "   ";
            LanguageC<Base>::printStaticIndexArray(_cache, os.str(), cols);

            maxNnzIndex = i;
        }
    }

    maxNnzIndex++;
    nnzs.resize(maxNnzIndex);

    auto makeArrayOfArrays = [&, this](const std::string& name) {
        _cache << "   static " << LanguageC<Base>::U_INDEX_TYPE << " const * const " << name << "[" << maxNnzIndex
               << "] = {";
        for (size_t i = 0; i < size_t(maxNnzIndex); i++) {
            if (i > 0) {
                _cache << ", ";
            }
            if (sparsities[i].rows.empty()) {
                _cache << "0";
            } else {
                _cache << name << i;
            }
        }
        _cache << "};\n";
    };

    if (maxNnzIndex > 0) {
        makeArrayOfArrays("rows");
        makeArrayOfArrays("cols");

        _cache << "   ";
        LanguageC<Base>::printStaticIndexArray(_cache, "nnzs", nnzs);

        _cache << "\n";

        _cache << "   if(i < " << maxNnzIndex << ") {\n"
                "      *row = rows[i];\n"
                "      *col = cols[i];\n"
                "      *nnz = nnzs[i];\n"
                "   } else {\n"
                "      *row = 0;\n"
                "      *col = 0;\n"
                "      *nnz = 0;\n"
                "   }\n";
    } else {
        _cache << "   *row = 0;\n"
                "   *col = 0;\n"
                "   *nnz = 0;\n";
    }

    _cache << "}\n";
}

template<class Base>
void ModelCSourceGen<Base>::generateSparsity1DSource2(const std::string& function,
                                                      const std::map<size_t, std::vector<size_t> >& elements) {
    LanguageC<Base>::printFunctionDeclaration(_cache, "void", function, {"unsigned long pos",
                                                                         "unsigned long const** elements",
                                                                         "unsigned long* nnz"});
    _cache << " {\n";

    std::vector<size_t> nnzs(elements.empty()? 0: elements.rbegin()->first + 1);

    long long int maxNnzIndex = -1;

    for (const auto& it : elements) {
        // the size of each sparsity row
        const std::vector<size_t>& els = it.second;
        if (!els.empty()) {
            _cache << "   ";
            std::ostringstream os;
            os << "els" << it.first;
            LanguageC<Base>::printStaticIndexArray(_cache, os.str(), els);

            maxNnzIndex = it.first;
            nnzs[it.first] = els.size();
        }
    }

    maxNnzIndex++;
    nnzs.resize(maxNnzIndex);

    if (maxNnzIndex > 0) {
        _cache << "   static " << LanguageC<Base>::U_INDEX_TYPE << " const * const els[" << maxNnzIndex << "] = {";
        auto it = elements.begin();
        for (size_t i = 0; i < size_t(maxNnzIndex); i++) {
            if (i > 0) {
                _cache << ", ";
            }
            if (it == elements.end() || i != it->first) {
                _cache << "0";
            } else {
                _cache << "els" << i;
                ++it;
            }
        }
        _cache << "};\n";

        _cache << "   ";
        LanguageC<Base>::printStaticIndexArray(_cache, "nnzs", nnzs);

        _cache << "\n";

        _cache << "   if(pos < " << maxNnzIndex << ") {\n"
                "      *elements = els[pos];\n"
                "      *nnz = nnzs[pos];\n"
                "   } else {\n"
                "      *elements = 0;\n"
                "      *nnz = 0;\n"
                "   }\n";
    } else {
        _cache << "   *elements = 0;\n"
                "   *nnz = 0;\n";
    }

    _cache << "}\n";
}

template<class Base>
inline std::map<size_t, std::vector<std::set<size_t> > > ModelCSourceGen<Base>::determineOrderByCol(const std::map<size_t, std::vector<size_t> >& elements,
                                                                                                    const LocalSparsityInfo& sparsity) {
    return determineOrderByCol(elements, sparsity.rows, sparsity.cols);
}

template<class Base>
inline std::map<size_t, std::vector<std::set<size_t> > > ModelCSourceGen<Base>::determineOrderByCol(const std::map<size_t, std::vector<size_t> >& elements,
                                                                                                    const std::vector<size_t>& userRows,
                                                                                                    const std::vector<size_t>& userCols) {
    std::map<size_t, std::vector<std::set<size_t> > > userLocation;

    for (const auto& it : elements) {
        size_t col = it.first;
        const std::vector<size_t>& colElements = it.second;

        userLocation[col] = determineOrderByCol(col, colElements, userRows, userCols);
    }

    return userLocation;
}

template<class Base>
inline std::vector<std::set<size_t> > ModelCSourceGen<Base>::determineOrderByCol(size_t col,
                                                                                 const std::vector<size_t>& colElements,
                                                                                 const std::vector<size_t>& userRows,
                                                                                 const std::vector<size_t>& userCols) {
    std::vector<std::set<size_t> > userLocationCol(colElements.size());

    for (size_t er = 0; er < colElements.size(); er++) {
        size_t row = colElements[er];
        for (size_t e = 0; e < userRows.size(); e++) {
            if (userRows[e] == row && userCols[e] == col) {
                userLocationCol[er].insert(e);
                break;
            }
        }
    }

    return userLocationCol;
}

template<class Base>
inline std::map<size_t, std::vector<std::set<size_t> > > ModelCSourceGen<Base>::determineOrderByRow(const std::map<size_t, std::vector<size_t> >& elements,
                                                                                                    const LocalSparsityInfo& sparsity) {
    return determineOrderByRow(elements, sparsity.rows, sparsity.cols);
}

template<class Base>
inline std::map<size_t, std::vector<std::set<size_t> > > ModelCSourceGen<Base>::determineOrderByRow(const std::map<size_t, std::vector<size_t> >& elements,
                                                                                                    const std::vector<size_t>& userRows,
                                                                                                    const std::vector<size_t>& userCols) {
    std::map<size_t, std::vector<std::set<size_t> > > userLocation;

    for (const auto& it : elements) {
        size_t row = it.first;
        const std::vector<size_t>& rowsElements = it.second;
        userLocation[row] = determineOrderByRow(row, rowsElements, userRows, userCols);
    }

    return userLocation;
}

template<class Base>
inline std::vector<std::set<size_t> > ModelCSourceGen<Base>::determineOrderByRow(size_t row,
                                                                                 const std::vector<size_t>& rowElements,
                                                                                 const std::vector<size_t>& userRows,
                                                                                 const std::vector<size_t>& userCols) {
    std::vector<std::set<size_t> > userLocationRow(rowElements.size());

    for (size_t ec = 0; ec < rowElements.size(); ec++) {
        size_t col = rowElements[ec];
        for (size_t e = 0; e < userRows.size(); e++) {
            if (userCols[e] == col && userRows[e] == row) {
                userLocationRow[ec].insert(e);
                break;
            }
        }
    }

    return userLocationRow;
}

template<class Base>
void ModelCSourceGen<Base>::printFileStartPThreads(std::ostringstream& cache,
                                                   const std::string& baseTypeName) {
    cache << "\n";
    cache << CPPADCG_PTHREAD_POOL_H_FILE << "\n";
    cache << "\n";
    cache << "typedef struct ExecArgStruct {\n"
            "   cppadcg_function_type func;\n"
            "   " << baseTypeName + " const *const * in;\n"
            "   " << baseTypeName + "* out[1];\n"
            "   struct LangCAtomicFun atomicFun;\n"
            "} ExecArgStruct;\n"
            "\n"
            "static void exec_func(void* arg) {\n"
            "   ExecArgStruct* eArg = (ExecArgStruct*) arg;\n"
            "   (*eArg->func)(eArg->in, eArg->out, eArg->atomicFun);\n"
            "}\n";
}

template<class Base>
void ModelCSourceGen<Base>::printFunctionStartPThreads(std::ostringstream& cache,
                                                       size_t size) {
    auto repeatFill = [&](const std::string& txt){
        cache << "{";
        for (size_t i = 0; i < size; ++i) {
            if (i != 0) cache << ", ";
            cache << txt;
        }
        cache << "};";
    };

    cache << "   ExecArgStruct* args[" << size << "];\n";
    cache << "   static cppadcg_thpool_function_type execute_functions[" << size << "] = ";
    repeatFill("exec_func");
    cache << "\n";
    cache << "   static float ref_elapsed[" << size << "] = ";
    repeatFill("0");
    cache << "\n";
    cache << "   static float elapsed[" << size << "] = ";
    repeatFill("0");
    cache << "\n"
            "   static int order[" << size << "] = {";
    for (size_t i = 0; i < size; ++i) {
        if (i != 0) cache << ", ";
        cache << i;
    }
    cache << "};\n"
            "   static int job2Thread[" << size << "] = ";
    repeatFill("-1");
    cache << "\n"
            "   static int last_elapsed_changed = 1;\n"
            "   unsigned int nBench = cppadcg_thpool_get_n_time_meas();\n"
            "   static unsigned int n_meas = 0;\n"
            "   int do_benchmark = " << (size > 0 ? "(n_meas < nBench && !cppadcg_thpool_is_disabled())" : "0") << ";\n"
            "   float* elapsed_p = do_benchmark ? elapsed : NULL;\n";
}

template<class Base>
void ModelCSourceGen<Base>::printFunctionEndPThreads(std::ostringstream& cache,
                                                     size_t size) {
    cache << "   cppadcg_thpool_add_jobs(execute_functions, (void**) args, ref_elapsed, elapsed_p, order, job2Thread, " << size << ", last_elapsed_changed" << ");\n"
            "\n"
            "   cppadcg_thpool_wait();\n"
            "\n"
            "   for(i = 0; i < " << size << "; ++i) {\n"
            "      free(args[i]);\n"
            "   }\n"
            "\n"
            "   if(do_benchmark) {\n"
            "      cppadcg_thpool_update_order(ref_elapsed, n_meas, elapsed, order, " << size << ");\n"
            "      n_meas++;\n"
            "   } else {\n"
            "      last_elapsed_changed = 0;\n"
            "   }\n";
}

template<class Base>
void ModelCSourceGen<Base>::printFileStartOpenMP(std::ostringstream& cache) {
    cache << CPPADCG_OPENMP_H_FILE << "\n"
            "#include <omp.h>\n"
            "#include <stdio.h>\n"
            "#include <time.h>\n";
}

template<class Base>
void ModelCSourceGen<Base>::printFunctionStartOpenMP(std::ostringstream& cache,
                                                     size_t size) {
    cache << "\n"
            "   enum omp_sched_t old_kind;\n"
            "   int old_modifier;\n"
            "   int enabled = !cppadcg_openmp_is_disabled();\n"
            "   int verbose = cppadcg_openmp_is_verbose();\n"
            "   struct timespec start[" << size << "];\n"
            "   struct timespec end[" << size << "];\n"
            "   int thread_id[" << size << "];\n"
            "   unsigned int n_threads = cppadcg_openmp_get_threads();\n"
            "   if(n_threads > " << size << ")\n"
            "      n_threads = " << size << ";\n"
            "\n"
            "   if(enabled) {\n"
            "      omp_get_schedule(&old_kind, &old_modifier);\n"
            "      cppadcg_openmp_apply_scheduler_strategy();\n"
            "   }\n";
}

template<class Base>
void ModelCSourceGen<Base>::printLoopStartOpenMP(std::ostringstream& cache,
                                                 size_t size) {
    cache <<"#pragma omp parallel for private(outLocal) schedule(runtime) if(enabled) num_threads(n_threads)\n"
            "   for(i = 0; i < " << size << "; ++i) {\n"
            "      int info;\n"
            "      if(verbose) {\n"
            "         thread_id[i] = omp_get_thread_num();\n"
            "         info = clock_gettime(CLOCK_MONOTONIC, &start[i]);\n"
            "         if(info != 0) {\n"
            "            start[i].tv_sec = 0;\n"
            "            start[i].tv_nsec = 0;\n"
            "            end[i].tv_sec = 0;\n"
            "            end[i].tv_nsec = 0;\n"
            "         }\n"
            "      }\n"
            "\n";
}

template<class Base>
void ModelCSourceGen<Base>::printLoopEndOpenMP(std::ostringstream& cache,
                                               size_t size) {
    cache <<"\n"
            "      if(verbose) {\n"
            "         if(info == 0) {\n"
            "            info = clock_gettime(CLOCK_MONOTONIC, &end[i]);\n"
            "            if(info != 0) {\n"
            "               end[i].tv_sec = 0;\n"
            "               end[i].tv_nsec = 0;\n"
            "            }\n"
            "         }\n"
            "      }\n"
            "   }\n"
            "\n"
            "   if(enabled) {\n"
            "      omp_set_schedule(old_kind, old_modifier);\n"
            "   }\n"
            "\n"
            "   if(verbose) {\n"
            "      struct timespec diff;\n"
            "      for (i = 0; i < " << size << "; ++i) {\n"
            "         if ((end[i].tv_nsec - start[i].tv_nsec) < 0) {\n"
            "            diff.tv_sec = end[i].tv_sec - start[i].tv_sec - 1;\n"
            "            diff.tv_nsec = end[i].tv_nsec - start[i].tv_nsec + 1000000000;\n"
            "         } else {\n"
            "            diff.tv_sec = end[i].tv_sec - start[i].tv_sec;\n"
            "            diff.tv_nsec = end[i].tv_nsec - start[i].tv_nsec;\n"
            "         }\n"
            "         fprintf(stdout, \"## Thread %i, Job %li, started at %ld.%.9ld, ended at %ld.%.9ld, elapsed %ld.%.9ld\\n\",\n"
            "                 thread_id[i], i, start[i].tv_sec, start[i].tv_nsec, end[i].tv_sec, end[i].tv_nsec, diff.tv_sec, diff.tv_nsec);\n"
            "      }\n"
            "   }\n";

}

template<class Base>
void ModelCSourceGen<Base>::startingJob(const std::string& jobName,
                                        const JobType& type) {
    if (_jobTimer != nullptr)
        _jobTimer->startingJob(jobName, type);
}

template<class Base>
inline void ModelCSourceGen<Base>::finishedJob() {
    if (_jobTimer != nullptr)
        _jobTimer->finishedJob();
}

/**
 *
 * Specializations
 */
template<>
inline std::string ModelCSourceGen<double>::baseTypeName() {
    return "double";
}

template<>
inline std::string ModelCSourceGen<float>::baseTypeName() {
    return "float";
}

} // END cg namespace
} // END CppAD namespace

#endif