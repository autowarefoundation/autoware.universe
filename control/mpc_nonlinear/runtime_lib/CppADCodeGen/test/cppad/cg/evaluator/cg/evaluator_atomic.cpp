/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2017 Ciengis
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
#include "cppad/cg/evaluator/CppADCGEvaluatorTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

void testModel(const std::vector<AD<double> >& x,
               std::vector<AD<double> >& y) {
    // model
    y[0] = x[0] + x[1];
    y[1] = y[0] + 1.;
    y[2] = 1. + y[1];
}

TEST_F(CppADCGEvaluatorTest, Atomic) {
    using ADCG = AD<CGD>;

    std::vector<AD<double>> ax(2);
    std::vector<AD<double>> ay(3);
    for (size_t j = 0; j < ax.size(); j++) {
        ax[j] = j + 2;
    }

    checkpoint<double> atomicFun("func", testModel, ax, ay); // the normal atomic function
    CGAtomicFun<double> atomic(atomicFun, ax); // a wrapper used to tape with CG<Base>

    ModelType model = [&](const std::vector<CGD>& x) {
        // independent variables
        std::vector<ADCG> ax(x.size());
        for (size_t j = 0; j < ax.size(); j++) {
            ax[j] = x[j];
        }

        CppAD::Independent(ax);

        // dependent variable vector
        std::vector<ADCG> ay(3);

        atomic(ax, ay);

        ADFun<CGD> fun;
        fun.Dependent(ay);

        std::vector<CGD> y = fun.Forward(0, x);

        return y;
    };

    this->testCG(model, std::vector<double>{0.5, 1.5});
}
