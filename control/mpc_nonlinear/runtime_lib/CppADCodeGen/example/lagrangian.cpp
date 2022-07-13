/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
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

#include <iosfwd>
#include <vector>
#include <cppad/cg.hpp>

using namespace CppAD;
using namespace CppAD::cg;

int main(void) {
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    size_t nCtr = 2; // number of constraints
    size_t nVar = 3; // number of decision variables

    std::vector<ADCG> v(nCtr + nVar); // independent variable vector
    Independent(v);

    std::vector<ADCG> x(nVar); // decision variables
    for (size_t i = 0; i < nVar; ++i)
        x[i] = v[i];
    std::vector<ADCG> lam(nCtr); // Lagrangian multipliers
    for (size_t i = 0; i < nCtr; ++i)
        lam[i] = v[nVar + i];

    // dependent variable vector
    std::vector<ADCG> lag(1);

    /**
     * the model of the Lagrangian
     */
    lag[0] = 2 * x[0] * x[0] + 3 * x[1] * x[1] + 4 * x[2] * x[2]; // objective
    lag[0] += lam[0] * (x[1] - 2 * x[2]); // constraint 1
    lag[0] += lam[1] * (x[0] * x[2]); // constraint 2

    ADFun<CGD> fun(v, lag);

    /***************************************************************************
     *                       Create the dynamic library
     *                  (generates and compiles source code)
     **************************************************************************/
    /**
     * Determine Hessian sparsity pattern
     */
    // determine the sparsity pattern
    using SparType = std::vector<std::set<size_t> >;
    SparType sparsity = CppAD::cg::hessianSparsitySet<SparType, CGD>(fun); // this could be improved

    size_t n = fun.Domain();
    assert(sparsity.size() == n);

    std::vector<size_t> rows, cols;
    for (size_t i = 0; i < nVar; i++) {
        for (size_t j : sparsity[i]) {
            if (j >= nVar) break;
            rows.push_back(i);
            cols.push_back(j);
        }
    }

    /**
     * Create the dynamic library
     * (generates and compiles source code)
     */

    // generates source code
    ModelCSourceGen<double> cgen(fun, "Lagrangian");
    cgen.setCreateSparseHessian(true);
    cgen.setCustomSparseHessianElements(rows, cols);
    ModelLibraryCSourceGen<double> libgen(cgen);

    // compile source code
    DynamicModelLibraryProcessor<double> p(libgen);

    GccCompiler<double> compiler;
    std::unique_ptr<DynamicLib<double>> dynamicLib = p.createDynamicLibrary(compiler);

    // save to files (not really required)
    SaveFilesModelLibraryProcessor<double> p2(libgen);
    p2.saveSources();

    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/
    std::unique_ptr<GenericModel<double>> model = dynamicLib->model("Lagrangian");
    std::vector<double> V(v.size());
    V[0] = 2.5;
    V[1] = 3.5;
    V[2] = 4.5;
    V[3] = 5.5; // multiplier
    V[4] = 6.5; // multiplier
    std::vector<double> w(1, 1.0);
    std::vector<double> hess;
    std::vector<size_t> row, col;

    model->SparseHessian(V, w, hess, row, col);

    // print out the result
    for (size_t i = 0; i < hess.size(); ++i)
        std::cout << "(" << row[i] << "," << col[i] << ") " << hess[i] << std::endl;

}
