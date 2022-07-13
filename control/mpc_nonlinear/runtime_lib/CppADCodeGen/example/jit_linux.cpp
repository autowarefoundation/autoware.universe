/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
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

#include <vector>
#include <cppad/cg.hpp>
#include <cppad/cg/model/llvm/llvm.hpp>

using namespace CppAD;
using namespace CppAD::cg;

int main() {
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    // independent variable vector
    std::vector<ADCG> x(2);
    Independent(x);

    // dependent variable vector
    std::vector<ADCG> y(1);

    // the model equation
    ADCG a = x[0] / 1. + x[1] * x[1];
    y[0] = a / 2;

    ADFun<CGD> fun(x, y);

    /***************************************************************************
     *                       Create the Model library
     *             (generates and prepare the source code for JIT)
     **************************************************************************/
    // generates source code
    ModelCSourceGen<double> cgen(fun, "model");
    cgen.setCreateJacobian(true);
    cgen.setCreateForwardOne(true);
    cgen.setCreateReverseOne(true);
    cgen.setCreateReverseTwo(true);
    ModelLibraryCSourceGen<double> libcgen(cgen);

    // JIT compile source code
    LlvmModelLibraryProcessor<double> p(libcgen);

    std::unique_ptr<LlvmModelLibrary<double>> llvmModelLib = p.create();

    // save to files (not really required)
    SaveFilesModelLibraryProcessor<double> p2(libcgen);
    p2.saveSources();

    /***************************************************************************
     *                       Use the JIT library
     **************************************************************************/

    std::unique_ptr<GenericModel<double>> model = llvmModelLib->model("model");
    std::vector<double> xv(x.size());
    xv[0] = 2.5;
    xv[1] = 3.5;
    std::vector<double> jac = model->Jacobian(xv);

    // print out the result
    std::cout << jac[0] << " " << jac[1] << std::endl;
}
