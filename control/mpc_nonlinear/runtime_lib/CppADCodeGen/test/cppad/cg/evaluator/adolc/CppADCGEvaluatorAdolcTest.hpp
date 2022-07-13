#ifndef CPPAD_CG_TEST_CPPADCGEVALUATORADOLCTEST_INCLUDED
#define	CPPAD_CG_TEST_CPPADCGEVALUATORADOLCTEST_INCLUDED
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
#include "CppADCGTest.hpp"
#include <cppad/cg/evaluator/evaluator_adolc.hpp>
#include <adolc/drivers/drivers.h>

namespace CppAD {
namespace cg {

class CppADCGEvaluatorAdolcTest : public CppADCGTest {
public:
    using ModelType = std::function<std::vector<CGD> (const std::vector<CGD>& x) >;

public:

    inline CppADCGEvaluatorAdolcTest(bool verbose = false,
                                     bool printValues = false) :
        CppADCGTest(verbose, printValues) {
    }

protected:

    inline void test(ModelType& model,
                     const std::vector<double>& testValues) {
        using std::vector;

        int tape = 0;

        CodeHandler<double> handlerOrig;

        // independent variable vector
        std::vector<CGD> xOrig(testValues.size());
        handlerOrig.makeVariables(xOrig);
        for (size_t j = 0; j < xOrig.size(); j++)
            xOrig[j].setValue(testValues[j]);

        const std::vector<CGD> yOrig = model(xOrig);

        /**
         * Test with adolc
         */

        // independents
        std::vector<adouble> xNew(xOrig.size());
        trace_on(tape);
        for (size_t i = 0; i < xOrig.size(); i++)
            xNew[i] <<= testValues[i];

        // model
        Evaluator<Base, Base, adouble> evaluator(handlerOrig);
        std::vector<adouble> yNew = evaluator.evaluate(xNew, yOrig);

        // dependents
        std::vector<double> rhsOut(yNew.size());
        for (size_t i = 0; i < yNew.size(); i++)
            yNew[i] >>= rhsOut[i];

        trace_off();

        ASSERT_EQ(yNew.size(), yOrig.size());
        for (size_t i = 0; i < yOrig.size(); i++) {
            ASSERT_NEAR(rhsOut[i], yOrig[i].getValue(), std::numeric_limits<Base>::epsilon()*100);
        }

        // evaluate the tape
        std::vector<Base> yBase(yOrig.size());
        std::vector<double> testValues2 = testValues; // must not be constant
        int retcode = ::function(tape, yBase.size(), testValues2.size(), testValues2.data(), yBase.data());

        ASSERT_TRUE(retcode >= 0);

        ASSERT_EQ(yBase.size(), yOrig.size());
        for (size_t i = 0; i < yOrig.size(); i++) {
            ASSERT_NEAR(yBase[i], yOrig[i].getValue(), std::numeric_limits<Base>::epsilon()*100);
        }
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
