#ifndef CPPAD_CG_TEST_PENDULUM_INCLUDED
#define CPPAD_CG_TEST_PENDULUM_INCLUDED
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

namespace CppAD {
namespace cg {

template<class Base>
inline CppAD::ADFun<Base>* Pendulum2D(std::vector<DaeVarInfo>& daeVar) {
    using namespace CppAD;
    using namespace std;
    using ADB = CppAD::AD<Base>;

    std::vector<ADB> U(11);
    Independent(U);

    ADB x = U[0];
    ADB y = U[1];
    ADB vx = U[2]; // auxiliary variable
    ADB vy = U[3]; // auxiliary variable
    ADB T = U[4]; // tension
    ADB L = U[5]; // length  (parameter)
    ADB t = U[6]; // time (not really used)

    ADB dxdt = U[7];
    ADB dydt = U[8];
    ADB dvxdt = U[9]; // auxiliary variable
    ADB dvydt = U[10]; // auxiliary variable

    daeVar.resize(U.size());
    daeVar[0] = DaeVarInfo("x");
    daeVar[1] = DaeVarInfo("y");
    daeVar[2] = DaeVarInfo("w"); // vx
    daeVar[3] = DaeVarInfo("z"); // vy
    daeVar[4] = DaeVarInfo("T");
    daeVar[5] = DaeVarInfo("l");
    daeVar[5].makeConstant();
    daeVar[6].makeIntegratedVariable();
    daeVar[7] = 0;
    daeVar[8] = 1;
    daeVar[9] = 2;
    daeVar[10] = 3;

    double g = 9.80665; // gravity constant

    // dependent variable vector 
    std::vector<ADB> Z(5);
    Z[0] = dxdt - vx; // dx/dt =
    //Z[0] = CppAD::CondExpLe<Base>(Z[0], 0, Z[0], 0);
    Z[1] = dydt - vy; // dy/dt =
    Z[2] = dvxdt - T * x; // dvx/dt =
    Z[3] = dvydt - (T * y - g); // dvy/dt =
    Z[4] = x * x + y * y - L * L;

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<Base> (U, Z);
}

template<class Base>
inline CppAD::ADFun<Base>* Pendulum3D() {
    using namespace CppAD;
    using namespace std;
    using ADB = CppAD::AD<Base>;

    std::vector<ADB> U(13);
    Independent(U);

    ADB x = U[0];
    ADB y = U[1];
    ADB z = U[2];
    ADB vx = U[3]; // auxiliary variable
    ADB vy = U[4]; // auxiliary variable
    ADB vz = U[5]; // auxiliary variable
    ADB T = U[6]; // tension
    ADB dxdt = U[7];
    ADB dydt = U[8];
    ADB dzdt = U[9];
    ADB dvxdt = U[10]; // auxiliary variable
    ADB dvydt = U[11]; // auxiliary variable
    ADB dvzdt = U[12]; // auxiliary variable

    double g = 9.80665; // gravity constant
    double L = 1.0; // fixed length

    // dependent variable vector 
    std::vector<ADB> Z(7);
    Z[0] = dxdt - vx; // dx/dt =
    Z[1] = dydt - vy; // dy/dt =
    Z[2] = dzdt - vz; // dz/dt =
    Z[3] = dvxdt - T * x; // dvx/dt =
    Z[4] = dvydt - (T * y - g); // dvy/dt =
    Z[5] = dvzdt - T * z; // dvz/dt =
    Z[6] = x * x + y * y + z * z - L * L;

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<Base> (U, Z);
}

} // END cg namespace
} // END CppAD namespace

#endif