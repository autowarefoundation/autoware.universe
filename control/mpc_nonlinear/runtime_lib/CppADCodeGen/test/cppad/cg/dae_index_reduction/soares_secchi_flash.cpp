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
#include <cppad/cg/dae_index_reduction/soares_secchi.hpp>

#include "CppADCGIndexReductionTest.hpp"
#include "model/flash.hpp"

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;


TEST_F(IndexReductionTest, SoaresSecchiFlash) {
    using CGD = CG<double>;

    std::vector<double> x(15);
    x[0] = 2.5;// nEthanol
    x[1] = 6.4;// nWater
    x[2] = 91;// T
    x[3] = 0.53;// yWater
    x[4] = 0.47;// yEthanol
    x[5] = 6.7;// FV
    x[6] = 500;// Q
    x[7] = 10;// F_feed
    x[8] = 1;// p
    x[9] = 0.5;// xFEthanol
    x[10] = 50;// T_feed

    x[11] = 0;// time

    x[12] = 0;// D__nEthanol__Dt
    x[13] = 0;// D__nWater__Dt
    x[14] = 0;// D__T__Dt

    std::vector<DaeVarInfo> daeVar;
    // create f: U -> Z and vectors used for derivative calculations
    ADFun<CGD>* fun = Flash<CGD> (daeVar, x);

    std::vector<std::string> eqName; // empty

    SoaresSecchi<double> soaresSecchi(*fun, daeVar, eqName, x);
    soaresSecchi.setVerbosity(Verbosity::High);

    std::vector<DaeVarInfo> newDaeVar;
    std::vector<DaeEquationInfo> equationInfo;
    std::unique_ptr<ADFun<CGD>> reducedFun;
    ASSERT_NO_THROW(reducedFun = soaresSecchi.reduceIndex(newDaeVar, equationInfo));

    ASSERT_TRUE(reducedFun != nullptr);

    ASSERT_EQ(size_t(2), soaresSecchi.getStructuralIndex());

    delete fun;
}
