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
#define CPPAD_CG_DAE_VERBOSE
#include <cppad/cg/dae_index_reduction/dummy_deriv.hpp>

#include "CppADCGIndexReductionTest.hpp"
#include "model/distillation.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(IndexReductionTest, DummyDerivDistillation) {
    using namespace std;

    std::vector<double> x(81);
    x[0] = 35250;
    x[1] = 11600;
    x[2] = 12400;
    x[3] = 12800;
    x[4] = 13600;
    x[5] = 14400;
    x[6] = 15200;
    x[7] = 64000;
    x[8] = 44750;
    x[9] = 8400;
    x[10] = 7600;
    x[11] = 7200;
    x[12] = 6400;
    x[13] = 5600;
    x[14] = 4800;
    x[15] = 16000;
    x[16] = 360;
    x[17] = 360;
    x[18] = 360;
    x[19] = 360;
    x[20] = 360;
    x[21] = 360;
    x[22] = 360;
    x[23] = 360;
    x[24] = 0.5;
    x[25] = 0.5;
    x[26] = 0.5;
    x[27] = 0.5;
    x[28] = 0.5;
    x[29] = 0.5;
    x[30] = 0.5;
    x[31] = 0.5;
    x[32] = 0.5;
    x[33] = 0.5;
    x[34] = 0.5;
    x[35] = 0.5;
    x[36] = 0.5;
    x[37] = 0.5;
    x[38] = 0.5;
    x[39] = 0.5;
    x[40] = 8;
    x[41] = 8;
    x[42] = 8;
    x[43] = 8;
    x[44] = 8;
    x[45] = 8;
    x[46] = 8;
    x[47] = 150;
    x[48] = 250;
    x[49] = 0.1;
    x[50] = 2.5;
    x[51] = 4;
    x[52] = 30;
    x[53] = 1;
    x[54] = 0.7;
    x[55] = 366;
    x[56] = 0;
    x[57] = 0;
    x[58] = 0;
    x[59] = 0;
    x[60] = 0;
    x[61] = 0;
    x[62] = 0;
    x[63] = 0;
    x[64] = 0;
    x[65] = 0;
    x[66] = 0;
    x[67] = 0;
    x[68] = 0;
    x[69] = 0;
    x[70] = 0;
    x[71] = 0;
    x[72] = 0;
    x[73] = 0;
    x[74] = 0;
    x[75] = 0;
    x[76] = 0;
    x[77] = 0;
    x[78] = 0;
    x[79] = 0;
    x[80] = 0;

    std::vector<DaeVarInfo> daeVar;
    
    std::vector<std::string> eqName; // empty

    // create f: U -> Z and vectors used for derivative calculations
    std::unique_ptr<ADFun<CGD> > fun(Distillation<CGD > (daeVar, x));

    std::vector<double> normVar(daeVar.size(), 1.0);
    std::vector<double> normEq(fun->Range(), 1.0);

    Pantelides<double> pantelides(*fun, daeVar, eqName, x);
    DummyDerivatives<double> dummyD(pantelides, x, normVar, normEq);
    dummyD.setAvoidConvertAlg2DifVars(false);
    dummyD.setGenerateSemiExplicitDae(true);
    dummyD.setReduceEquations(true);
    dummyD.setReorder(true);
    dummyD.setVerbosity(Verbosity::Low);

    std::vector<DaeVarInfo> newDaeVar;
    std::vector<DaeEquationInfo> newEqInfo;
    std::unique_ptr<ADFun<CGD>> reducedFun;
    ASSERT_NO_THROW(reducedFun = dummyD.reduceIndex(newDaeVar, newEqInfo));

    ASSERT_TRUE(reducedFun != nullptr);

    ASSERT_EQ(size_t(2), pantelides.getStructuralIndex());
}
