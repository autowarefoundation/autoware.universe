#ifndef TEST_SOLVE_INCLUDED
#define	TEST_SOLVE_INCLUDED
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
#include "CppADCGTest.hpp"

namespace CppAD {
namespace cg {

class CppADCGSolveTest : public CppADCGTest {
public:

    inline CppADCGSolveTest(bool verbose = false,
                            bool printValues = false) :
        CppADCGTest(verbose, printValues) {
    }

protected:

    inline void test_solve(CppAD::ADFun<CGD>& fun,
                           size_t expressionIndex,
                           size_t indIndex,
                           const std::vector<ADCGD>& testValues) {

        std::vector<double> testValuesD(testValues.size());
        for (size_t i = 0; i < testValues.size(); i++) {
            testValuesD[i] = CppAD::Value(CppAD::Var2Par(testValues[i])).getValue();
        }

        test_solve(fun, expressionIndex, indIndex, testValuesD);
    }

    inline void test_solve(CppAD::ADFun<CGD>& fun,
                           size_t expressionIndex,
                           size_t indIndex,
                           const std::vector<double>& testValues) {

        using std::vector;

        size_t m = fun.Range();
        size_t n = fun.Domain();

        assert(expressionIndex < m);
        assert(indIndex < n);
        ASSERT_EQ(testValues.size(), n);

        // evaluate the dependent values (equation residuals)
        const vector<double> depValues = calculateDependentForward0(fun, testValues);

        ASSERT_TRUE(nearEqual(depValues[expressionIndex], 0.0, 1e-4, 1e-8));

        // generate the operation graph
        CodeHandler<double> handler;
        vector<CGD> indVars(n);
        handler.makeVariables(indVars);

        vector<CGD> dep = fun.Forward(0, indVars);

        /**
         * solve
         */
        CGD solution = handler.solveFor(*dep[expressionIndex].getOperationNode(), *indVars[indIndex].getOperationNode());

        if (verbose_) {
            printModel(solution);
        }

        /**
         * create a new tape
         */
        vector<CGD> newDep(1);
        newDep[0] = solution;

        // new independent vector (without one variable)
        vector<AD<double> > newIndep(n - 1);
        for (size_t i = 0; i < indIndex; i++) {
            newIndep[i] = testValues[i];
        }
        for (size_t i = indIndex + 1; i < n; i++) {
            newIndep[i - 1] = testValues[i];
        }

        // create a longer independent vector to use in the evaluator
        vector<AD<double> > newIndepLong(n);
        for (size_t i = 0; i < indIndex; i++) {
            newIndepLong[i] = newIndep[i];
        }
        newIndepLong[indIndex] = std::numeric_limits<double>::quiet_NaN();
        for (size_t i = indIndex + 1; i < n; i++) {
            newIndepLong[i] = newIndep[i - 1];
        }

        // determine the result
        Evaluator<double, double> evaluator(handler);
        vector<AD<double> > result = evaluator.evaluate(newIndepLong, newDep);

        double resultVal = CppAD::Value(CppAD::Var2Par(result[0]));

        ASSERT_TRUE(nearEqual(resultVal, testValues[indIndex], 10e-4, 10e-4));
    }

    inline std::vector<double> calculateDependentForward0(CppAD::ADFun<CGD>& fun,
                                                          const std::vector<double>& testValues) {
        using std::vector;

        size_t m = fun.Range();
        size_t n = fun.Domain();

        vector<CGD> indVars(n);
        for (size_t i = 0; i < n; ++i) {
            indVars[i] = testValues[i];
        }

        vector<CGD> dep = fun.Forward(0, indVars);

        vector<double> depVals(m);
        for (size_t i = 0; i < m; ++i) {
            depVals[i] = dep[i].getValue();
        }

        return depVals;
    }

    inline void printModel(CGD& dep) {
        printExpression(dep);
        std::cout << std::endl;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
