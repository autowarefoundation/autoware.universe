#ifndef CPPAD_CG_TEST_MATTSSON_LINEAR_INCLUDED
#define CPPAD_CG_TEST_MATTSSON_LINEAR_INCLUDED
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

namespace CppAD {
namespace cg {

/**
 * Second order linear DAE system
 *
 * S. Mattsson and G. Söderlind, Index reduction in differential-algebraic
 * equations using dummy derivatives, SIAM J. Sci. Comput. 14, 677–692 (1993).
 */
template<class Base>
inline CppAD::ADFun<Base>* MattssonLinear(std::vector<DaeVarInfo>& daeVar,
                                          std::vector<double>& x) {
    using namespace CppAD;
    using namespace std;
    using ADB = CppAD::AD<Base>;

    daeVar.resize(19);
    x.resize(19);

    /**
     * Variable description
     */
    daeVar[0] = DaeVarInfo("x1");
    daeVar[1] = DaeVarInfo("x2");
    daeVar[2] = DaeVarInfo("x3");
    daeVar[3] = DaeVarInfo("x4");
    daeVar[4] = DaeVarInfo("x5"); // dx1dt
    daeVar[5] = DaeVarInfo("x6"); // dx2dt
    daeVar[6] = DaeVarInfo("x7"); // dx3dt
    daeVar[7].makeConstant();
    daeVar[8].makeConstant();
    daeVar[9].makeConstant();
    daeVar[10].makeConstant();
    daeVar[11].makeIntegratedVariable();
    daeVar[12] = 0;  // dx1dt
    daeVar[13] = 1; // dx2dt
    daeVar[14] = 2; // dx3dt
    daeVar[15] = 3; // dx4dt
    daeVar[16] = 4; // d(dx1dt)dt
    daeVar[17] = 5; // d(dx2dt)dt
    daeVar[18] = 6; // d(dx3dt)dt

    /**
     * Values
     */
    x[0] = 1; // x1
    x[1] = 1; // x2
    x[2] = 1; // x3
    x[3] = 1; // x4
    x[4] = 1; // x5 = dx1dt
    x[5] = 1; // x6 = dx2dt
    x[6] = 1; // x7 = dx3dt
    x[7] = 1; // u1
    x[8] = 1; // u2
    x[9] = 1; // u3
    x[10] = 1; // u4

    x[11] = 0; // time

    x[12] = x[4]; // dx1dt
    x[13] = x[5]; // dx2dt
    x[14] = x[6]; // dx3dt
    x[15] = 1; // dx4dt
    x[16] = 1; // d(dx1dt)dt
    x[17] = 1; // d(dx2dt)dt
    x[18] = 1; // d(dx3dt)dt

    /**
     * Model
     */
    std::vector<ADB> U(x.size());
    for (size_t i = 0; i < x.size(); i++) {
        U[i] = x[i];
    }
    Independent(U);

    ADB x1 = U[0];
    ADB x2 = U[1];
    ADB x3 = U[2];
    ADB x4 = U[3];
    ADB x5 = U[4]; // dx1dt
    ADB x6 = U[5]; // dx2dt
    ADB x7 = U[6]; // dx3dt
    ADB u1 = U[7];
    ADB u2 = U[8];
    ADB u3 = U[9];
    ADB u4 = U[10];

    ADB dx1dt = U[12];
    ADB dx2dt = U[13];
    ADB dx3dt = U[14];
    ADB dx4dt = U[15];

    ADB dx5dt = U[16]; // d(dx1dt)dt
    ADB dx6dt = U[17]; // d(dx2dt)dt
    ADB dx7dt = U[18]; // d(dx3dt)dt

    // dependent variable vector
    std::vector<ADB> res(7);

    res[0] = x5 - dx1dt;
    res[1] = x6 - dx2dt;
    res[2] = x7 - dx3dt;
    res[3] = x1 + x2 + u1;
    res[4] = x1 + x2 + x3 + u2;
    res[5] = dx3dt + x1 + x4 + u3;
    res[6] = 2 * dx5dt + dx6dt + dx7dt + dx4dt + u4;

    // create f: U -> res and vectors used for derivative calculations
    return new ADFun<Base> (U, res);
}

} // END cg namespace
} // END CppAD namespace

#endif