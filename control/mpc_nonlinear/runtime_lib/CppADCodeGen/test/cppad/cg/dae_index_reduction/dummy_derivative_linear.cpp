/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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
//#define CPPAD_CG_DAE_VERBOSE
#include <cppad/cg/dae_index_reduction/dummy_deriv.hpp>

#include "CppADCGIndexReductionTest.hpp"
#include "model/mattsson_linear.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(IndexReductionTest, DummyDerivMattsson) {
    using namespace std;

    std::vector<DaeVarInfo> daeVar;
    std::vector<double> x;

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<CGD>* fun = MattssonLinear<CGD> (daeVar, x);

    std::vector<double> normVar(daeVar.size(), 1.0);
    std::vector<double> normEq(7, 1.0);

    std::vector<std::string> eqName; // empty

    Pantelides<double> pantelides(*fun, daeVar, eqName, x);
    pantelides.setVerbosity(Verbosity::Low);

    DummyDerivatives<double> dummyD(pantelides, x, normVar, normEq);
    dummyD.setVerbosity(Verbosity::High);
    dummyD.setGenerateSemiExplicitDae(true);
    dummyD.setReduceEquations(true);

    std::vector<DaeVarInfo> newDaeVar;
    std::vector<DaeEquationInfo> newEqInfo;
    std::unique_ptr<ADFun<CGD>> reducedFun;
    ASSERT_NO_THROW(reducedFun = dummyD.reduceIndex(newDaeVar, newEqInfo));

    ASSERT_TRUE(reducedFun != nullptr);

    ASSERT_EQ(size_t(3), pantelides.getStructuralIndex());
    //ASSERT_EQ(10, reducedFun->Range());

    delete fun;
}