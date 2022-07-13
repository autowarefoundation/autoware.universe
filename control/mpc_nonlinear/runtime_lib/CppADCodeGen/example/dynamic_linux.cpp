/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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

#include <vector>
#include <cppad/cg.hpp>

using namespace CppAD;
using namespace CppAD::cg;

const int N = 2;
const std::string LIBRARY_NAME = "./model_library";
const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;

void compileLibrary() {
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    // independent variable vector
    std::vector<ADCG> x(N);
    Independent(x);

    // dependent variable vector
    std::vector<ADCG> y(1);

    // the model equation
    ADCG a = x[0] / 1. + x[1] * x[1];
    y[0] = a / 2;

    ADFun<CGD> fun(x, y);

    /***************************************************************************
     *                       Create the dynamic library
     *                  (generates and compiles source code)
     **************************************************************************/
    // generates source code
    ModelCSourceGen<double> cgen(fun, "model");
    cgen.setCreateJacobian(true);
    cgen.setCreateForwardOne(true);
    cgen.setCreateReverseOne(true);
    cgen.setCreateReverseTwo(true);
    ModelLibraryCSourceGen<double> libcgen(cgen);

    // compile source code
    DynamicModelLibraryProcessor<double> p(libcgen, LIBRARY_NAME);

    GccCompiler<double> compiler;
    bool loadLib = false;
    p.createDynamicLibrary(compiler, loadLib);

    // save to files (not really required)
    SaveFilesModelLibraryProcessor<double> p2(libcgen);
    p2.saveSources();
}

void useLibrary() {
    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/
    LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<GenericModel<double>> model = dynamicLib.model("model");
    std::vector<double> xv{2.5, 3.5};
    std::vector<double> jac = model->Jacobian(xv);

    // print out the result
    std::cout << jac[0] << " " << jac[1] << std::endl;
}

int main() {
    if (!system::isFile(LIBRARY_NAME_EXT)) {
        std::cout << "Creating a new library" << std::endl;
        compileLibrary();
    } else {
        std::cout << "Reusing existing library" << std::endl;
    }

    useLibrary();
}
