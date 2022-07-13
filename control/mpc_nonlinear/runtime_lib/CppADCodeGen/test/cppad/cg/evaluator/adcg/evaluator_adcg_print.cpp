/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2018 Ciengis
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
#include "../CppADCGEvaluatorTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGEvaluatorTest, Print) {
    CodeHandler<double> handler;

    std::vector<CG<double>> x(3);
    handler.makeVariables(x);

    std::vector<CG<double>> y(3);

    y[0] = makePrintValue("y[0]=", x[0] + x[1] + x[2], "\n");
    y[1] = makePrintValue("x[0]=", x[0], "\n") * 2;
    y[2] = makePrintValue("y[2]=", y[1], "\n") * 2;

    CppAD::cg::Evaluator<double, double, CppAD::AD<double>> evaluator(handler);

    std::vector<CppAD::AD<double>> xNew(x.size());
    CppAD::Independent(xNew);

    std::vector<CppAD::AD<double>> yNew(y.size());

    std::cout << "Evaluating..."<<std::endl;
    evaluator.evaluate(xNew, yNew, y);

    std::cout << "Using tape..."<<std::endl;
    CppAD::ADFun<double> fun(xNew, yNew);

    std::vector<double> x2(x.size());
    for(size_t i = 0; i< x2.size(); ++i)
        x2[i] = i + 1;

    std::vector<double> y2 = fun.Forward(0, x2);

    // validate results
    ASSERT_EQ(y2[0], x2[0] + x2[1] + x2[2]);
    ASSERT_EQ(y2[1], x2[0] * 2);
    ASSERT_EQ(y2[2], y2[1] * 2);
}
