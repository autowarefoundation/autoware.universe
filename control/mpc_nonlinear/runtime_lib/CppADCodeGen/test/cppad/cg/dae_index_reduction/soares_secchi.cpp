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
#include "model/pendulum.hpp"
#include "model/simple2d.hpp"

using namespace CppAD;
using namespace CppAD::cg;
using namespace std;

TEST_F(IndexReductionTest, SoaresSecchiSimple2D) {
    using CGD = CG<double>;

    std::vector<DaeVarInfo> daeVar;
    // create f: U -> Z and vectors used for derivative calculations
    ADFun<CGD>* fun = Simple2D<CGD> (daeVar);

    std::vector<double> x(daeVar.size());
    x[0] = -1.0; // x1
    x[1] = 1.0; // x2

    x[2] = 0.0; // time

    x[3] = 0.0; // dx1dt
    x[4] = 0.0; // dx2dt

    std::vector<std::string> eqName; // empty

    SoaresSecchi<double> soaresSecchi(*fun, daeVar, eqName, x);
    //soaresSecchi.setVerbosity(Verbosity::High);

    std::vector<DaeVarInfo> newDaeVar;
    std::vector<DaeEquationInfo> equationInfo;
    std::unique_ptr<ADFun<CGD>> reducedFun;
    ASSERT_NO_THROW(reducedFun = soaresSecchi.reduceIndex(newDaeVar, equationInfo));

    ASSERT_TRUE(reducedFun == nullptr);

    ASSERT_EQ(size_t(1), soaresSecchi.getStructuralIndex());

    delete fun;
}

TEST_F(IndexReductionTest, SoaresSecchiPendulum2D) {
    using CGD = CG<double>;

    std::vector<DaeVarInfo> daeVar;
    // create f: U -> Z and vectors used for derivative calculations
    ADFun<CGD>* fun = Pendulum2D<CGD> (daeVar);

    std::vector<double> x(daeVar.size());
    x[0] = -1.0; // x
    x[1] = 0.0; // y
    x[2] = 0.0; // vx
    x[3] = 0.0; // vy
    x[4] = 1.0; // Tension
    x[5] = 1.0; // length

    x[6] = 0.0; // time

    x[7] = 0.0; // dxdt
    x[8] = 0.0; // dydt
    x[9] = -1.0; // dvxdt
    x[10] = 9.80665; // dvydt
    
    std::vector<std::string> eqName; // empty

    SoaresSecchi<double> soaresSecchi(*fun, daeVar, eqName, x);
    //soaresSecchi.setVerbosity(Verbosity::High);

    std::vector<DaeVarInfo> newDaeVar;
    std::vector<DaeEquationInfo> equationInfo;
    std::unique_ptr<ADFun<CGD>> reducedFun;
    ASSERT_NO_THROW(reducedFun = soaresSecchi.reduceIndex(newDaeVar, equationInfo));

    ASSERT_TRUE(reducedFun != nullptr);

    ASSERT_EQ(size_t(3), soaresSecchi.getStructuralIndex());

    delete fun;
}

TEST_F(IndexReductionTest, SoaresSecchiPendulum3D) {
    using CGD = CG<double>;

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<CGD>* fun = Pendulum3D<CGD> ();

    std::vector<DaeVarInfo> daeVar(13);
    daeVar[7] = 0;
    daeVar[8] = 1;
    daeVar[9] = 2;
    daeVar[10] = 3;
    daeVar[11] = 4;
    daeVar[12] = 5;

    std::vector<double> x(13);
    x[0] = -1.0; // x
    x[1] = 0.0; // y
    x[2] = 0.0; // z
    x[3] = 0.0; // vx
    x[4] = 0.0; // vy
    x[5] = 0.0; // vz
    x[6] = 1.0; // Tension
    //x[7] = 1.0; // length
    x[7] = 0.0; // dxdt
    x[8] = 0.0; // dydt
    x[9] = 0.0; // dzdt
    x[10] = -1.0; // dvxdt
    x[11] = 9.80665; // dvydt
    x[12] = 0.0; // dvzdt
    
    std::vector<std::string> eqName; // empty
    
    SoaresSecchi<double> soaresSecchi(*fun, daeVar, eqName, x);
    //soaresSecchi.setVerbosity(Verbosity::High);

    std::vector<DaeVarInfo> newDaeVar;
    std::vector<DaeEquationInfo> equationInfo;
    std::unique_ptr<ADFun<CGD>> reducedFun;
    ASSERT_NO_THROW(reducedFun = soaresSecchi.reduceIndex(newDaeVar, equationInfo));

    ASSERT_TRUE(reducedFun != nullptr);

    ASSERT_EQ(size_t(3), soaresSecchi.getStructuralIndex());

    delete fun;
}
