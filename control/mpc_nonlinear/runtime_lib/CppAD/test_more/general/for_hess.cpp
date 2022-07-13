/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


/*
Old ForHess example now used just for valiadation testing
*/

# include <cppad/cppad.hpp>


bool ForHess(void)
{   bool ok = true;

    using namespace CppAD;
    using CppAD::exp;
    using CppAD::sin;
    using CppAD::cos;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    size_t i;

    // create independent variable vector with assigned values
    CPPAD_TESTVECTOR(double)      u0(3);
    CPPAD_TESTVECTOR(AD<double>) U(3);
    for(i = 0; i < 3; i++)
        U[i] = u0[i] = double(i+1);
    Independent( U );

    // define the function
    CPPAD_TESTVECTOR(AD<double>) Y(2);
    Y[0] = U[0] * exp( U[1] );
    Y[1] = U[1] * sin( U[2] );

    // create the function y = F(u)
    ADFun<double> F(U, Y);

    // formulas for the upper triangle of Hessian of F_0
    CPPAD_TESTVECTOR(double) H0(9);
    H0[0] = 0.;                    // d^2 y[0] / d_u[0] d_u[0]
    H0[1] = exp( u0[1] );          // d^2 y[0] / d_u[0] d_u[1]
    H0[2] = 0.;                    // d^2 y[0] / d_u[0] d_u[2]

    H0[4] = u0[0] * exp( u0[1] );  // d^2 y[0] / d_u[1] d_u[1]
    H0[5] = 0.;                    // d^2 y[0] / d_u[1] d_u[2]

    H0[8] = 0.;                    // d^2 y[0] / d_u[2] d_u[2]

    // formulas for the upper triangle of Hessian of F_1
    CPPAD_TESTVECTOR(double) H1(9);
    H1[0] = 0.;                    // d^2 Y[1] / d_U[0] d_U[0]
    H1[1] = 0.;                    // d^2 Y[1] / d_U[0] d_U[1]
    H1[2] = 0.;                    // d^2 Y[1] / d_U[0] d_U[2]

    H1[4] = 0.;                    // d^2 Y[1] / d_U[1] d_U[1]
    H1[5] = cos( u0[2] );          // d^2 Y[1] / d_U[1] d_U[2]

    H1[8] = - u0[1] * sin( u0[2] );// d^2 Y[1] / d_U[2] d_U[2]


    // Define U(t) = u0 + u1 t + u2 t^2 / 2
    CPPAD_TESTVECTOR(double) u1(3);
    CPPAD_TESTVECTOR(double) u2(3);
    for(i = 0; i < 3; i++)
        u1[i] = u2[i] = 0.;

    size_t j;
    for(i = 0; i < 3; i++)
    {   // diagonal of Hessians in i-th coordiante direction
        u1[i] = 1.;
        F.Forward(1, u1);
        CPPAD_TESTVECTOR(double) Di = F.Forward(2, u2);
        ok &= NearEqual( 2. * Di[0] , H0[ i + 3 * i], eps99, eps99);
        ok &= NearEqual( 2. * Di[1] , H1[ i + 3 * i], eps99, eps99);
        //
        for(j = i+1; j < 3; j++)
        {   // cross term in i and j direction
            u1[j] = 1.;
            F.Forward(1, u1);
            CPPAD_TESTVECTOR(double) Cij = F.Forward(2, u2);

            // diagonal of Hessian in j-th coordinate direction
            u1[i] = 0.;
            F.Forward(1, u1);
            CPPAD_TESTVECTOR(double) Dj = F.Forward(2, u2);

            // (i, j) elements of the Hessians
            double H0ij = Cij[0] - Di[0] - Dj[0];
            ok &= NearEqual( H0ij, H0[j + 3 * i], eps99, eps99);
            double H1ij = Cij[1] - Di[1] - Dj[1];
            ok &= NearEqual( H1ij, H1[j + 3 * i], eps99, eps99);

            // reset all components of u1 to zero
            u1[j] = 0.;
        }
    }

    return ok;
}
