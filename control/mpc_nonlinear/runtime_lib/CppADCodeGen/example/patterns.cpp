/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    // independent variable vector
    std::vector<ADCG> x(5, ADCG(1));
    Independent(x);

    // dependent variable vector 
    std::vector<ADCG> y(8);

    // temporary variables
    ADCG a, b;

    // the model    
    a = exp(3 * x[1]);

    b = 5 * x[0] * x[4];
    y[0] = a / 2 + b;
    // one equation not defined!
    y[1] = x[2] - b;

    b = 5 * x[1] * x[3];
    y[2] = a / 2 + b;
    y[3] = x[4] * x[1] + b;
    y[4] = x[3] - b;

    b = 5 * x[2] * x[2];
    y[5] = a / 2 + b;
    y[6] = x[4] * x[2] + b;
    y[7] = x[4] - b;

    ADFun<CGD> fun(x, y);

    /***************************************************************************
     *                             pattern detection
     *                                   and 
     *                             source generation
     **************************************************************************/

    /**
     * Define which dependent variables are related
     */
    std::vector<std::set<size_t> > relatedDep{
        {0, 2, 5},
        {3, 6},
        {1, 4, 7}
    };

    /**
     * generates source code
     */
    ModelCSourceGen<double> compModelH(fun, "model");
    compModelH.setCreateSparseJacobian(true);
    compModelH.setCreateSparseHessian(true);
    compModelH.setRelatedDependents(relatedDep);
    ModelLibraryCSourceGen<double> compDynH(compModelH);

    // save to files
    SaveFilesModelLibraryProcessor<double> p(compDynH);
    p.saveSources();
}
