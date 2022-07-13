#ifndef CPPAD_CG_TEST_CPPADCGEVALUATORTEST_INCLUDED
#define	CPPAD_CG_TEST_CPPADCGEVALUATORTEST_INCLUDED
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

namespace CppAD {
namespace cg {

class CppADCGEvaluatorTest : public CppADCGTest {
public:
    using ModelType = std::function<std::vector<CGD> (const std::vector<CGD>& x) >;

public:

    inline CppADCGEvaluatorTest(bool verbose = false,
                                bool printValues = false) :
        CppADCGTest(verbose, printValues) {
    }

protected:

    inline void test(ModelType& model,
                     const std::vector<double>& testValues) {
        using std::vector;

        CodeHandler<double> handlerOrig;

        // independent variable vector
        std::vector<CGD> xOrig(testValues.size());
        handlerOrig.makeVariables(xOrig);
        for (size_t j = 0; j < xOrig.size(); j++)
            xOrig[j].setValue(testValues[j]);

        const std::vector<CGD> yOrig = model(xOrig);

        /**
         * Test with scalars only
         */
        {
            Evaluator<Base, Base, CGD> evaluator(handlerOrig);

            std::vector<CGD> xNew(xOrig.size());
            for (size_t j = 0; j < xOrig.size(); j++)
                xNew[j] = testValues[j];

            std::vector<CGD> yNew = evaluator.evaluate(xNew, yOrig);

            ASSERT_EQ(yNew.size(), yOrig.size());
            for (size_t i = 0; i < yOrig.size(); i++) {
                ASSERT_EQ(yNew[i].getValue(), yOrig[i].getValue());
            }
        }

        /**
         * Test with active variables from CG
         */
        {
            testCG(testValues, yOrig);
        }

        /**
         * Test with active variables from CppAD
         */
        {
            std::vector<AD<Base> > xNew(xOrig.size());
            for (size_t j = 0; j < xOrig.size(); j++)
                xNew[j] = testValues[j];

            CppAD::Independent(xNew);

            //
            Evaluator<Base, Base, AD<Base> > evaluator(handlerOrig);
            std::vector<AD<Base> > yNew = evaluator.evaluate(xNew, yOrig);

            ASSERT_EQ(yNew.size(), yOrig.size());
            for (size_t i = 0; i < yOrig.size(); i++) {
                ASSERT_EQ(CppAD::Variable(yNew[i]), yOrig[i].isVariable());
            }

            // create the CppAD tape
            CppAD::ADFun<Base> fun;
            fun.Dependent(yNew);

            // evaluate the tape
            std::vector<Base> yBase = fun.Forward(0, testValues);

            ASSERT_EQ(yBase.size(), yOrig.size());
            for (size_t i = 0; i < yOrig.size(); i++) {
                ASSERT_EQ(yBase[i], yOrig[i].getValue());
            }
        }

        /**
         * Test with active variables from CppAD<CG>
         */
        {
            std::vector<ADCGD> xNew(xOrig.size());
            for (size_t j = 0; j < xOrig.size(); j++)
                xNew[j] = testValues[j];

            CppAD::Independent(xNew);

            //
            Evaluator<Base, CGD, ADCGD> evaluator(handlerOrig);
            std::vector<ADCGD> yNew = evaluator.evaluate(xNew, yOrig);

            ASSERT_EQ(yNew.size(), yOrig.size());
            for (size_t i = 0; i < yOrig.size(); i++) {
                ASSERT_EQ(CppAD::Variable(yNew[i]), yOrig[i].isVariable());
            }

            // create the CppAD tape
            CppAD::ADFun<CGD> fun;
            fun.Dependent(yNew);

            // evaluate the tape
            CodeHandler<double> handlerNew;
            std::vector<CGD> xNew2(xOrig.size());
            handlerNew.makeVariables(xNew2);
            for (size_t j = 0; j < xOrig.size(); j++)
                xNew2[j].setValue(testValues[j]);
            std::vector<CGD> yNew2 = fun.Forward(0, xNew2);

            ASSERT_EQ(yNew2.size(), yOrig.size());
            for (size_t i = 0; i < yOrig.size(); i++) {
                ASSERT_EQ(yNew2[i].isVariable(), yOrig[i].isVariable());
                ASSERT_EQ(yNew2[i].getValue(), yOrig[i].getValue());
            }
        }
    }

    inline void testCG(ModelType& model,
                       const std::vector<double>& testValues) {
        using std::vector;

        CodeHandler<double> handlerOrig;

        // independent variable vector
        std::vector<CGD> xOrig(testValues.size());
        handlerOrig.makeVariables(xOrig);
        for (size_t j = 0; j < xOrig.size(); j++)
            xOrig[j].setValue(testValues[j]);

        const std::vector<CGD> yOrig = model(xOrig);

        testCG(testValues, yOrig);
    }


    inline void testCG(const std::vector<double>& testValues,
                       const std::vector<CGD>& yOrig) {
        using std::vector;

        assert(yOrig.size() > 0);
        assert(yOrig[0].getOperationNode() != nullptr);
        assert(yOrig[0].getOperationNode()->getCodeHandler() != nullptr);
        CodeHandler<double>& handlerOrig = *yOrig[0].getOperationNode()->getCodeHandler();

        CodeHandler<double> handlerNew;

        std::vector<CGD> xNew(testValues.size());
        handlerNew.makeVariables(xNew);
        for (size_t j = 0; j < testValues.size(); j++)
            xNew[j].setValue(testValues[j]);

        Evaluator<Base, Base, CGD> evaluator(handlerOrig);
        std::vector<CGD> yNew = evaluator.evaluate(xNew, yOrig);

        ASSERT_EQ(yNew.size(), yOrig.size());
        for (size_t i = 0; i < yOrig.size(); i++) {
            ASSERT_EQ(yNew[i].isVariable(), yOrig[i].isVariable());
            ASSERT_EQ(yNew[i].getValue(), yOrig[i].getValue());
        }

    }

    inline void printModel(CodeHandler<double>& handler, CGD& dep) {
        LanguageC<double> langC("double");
        LangCDefaultVariableNameGenerator<double> nameGen;

        CppAD::vector<CGD> depv(1);
        depv[0] = dep;

        std::ostringstream code;
        handler.generateCode(code, langC, depv, nameGen);
        std::cout << code.str();
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
