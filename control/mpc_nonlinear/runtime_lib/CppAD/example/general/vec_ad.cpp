/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin vec_ad.cpp$$
$spell
    Vec
    Cpp
    cstddef
$$

$section AD Vectors that Record Index Operations: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cassert>

namespace {
    // return the vector x that solves the following linear system
    //    a[0] * x[0] + a[1] * x[1] = b[0]
    //    a[2] * x[0] + a[3] * x[1] = b[1]
    // in a way that will record pivot operations on the AD<double> tape
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) Vector;
    Vector Solve(const Vector &a , const Vector &b)
    {   using namespace CppAD;
        assert(a.size() == 4 && b.size() == 2);

        // copy the vector b into the VecAD object B
        VecAD<double> B(2);
        AD<double>    u;
        for(u = 0; u < 2; u += 1.)
            B[u] = b[ size_t( Integer(u) ) ];

        // copy the matrix a into the VecAD object A
        VecAD<double> A(4);
        for(u = 0; u < 4; u += 1.)
            A[u] = a [ size_t( Integer(u) ) ];

        // tape AD operation sequence that determines the row of A
        // with maximum absolute element in column zero
        AD<double> zero(0), one(1);
        AD<double> rmax = CondExpGt(fabs(a[0]), fabs(a[2]), zero, one);

        // divide row rmax by A(rmax, 0)
        A[rmax * 2 + 1]  = A[rmax * 2 + 1] / A[rmax * 2 + 0];
        B[rmax]          = B[rmax]         / A[rmax * 2 + 0];
        A[rmax * 2 + 0]  = one;

        // subtract A(other,0) times row A(rmax, *) from row A(other,*)
        AD<double> other   = one - rmax;
        A[other * 2 + 1]   = A[other * 2 + 1]
                           - A[other * 2 + 0] * A[rmax * 2 + 1];
        B[other]           = B[other]
                           - A[other * 2 + 0] * B[rmax];
        A[other * 2 + 0] = zero;

        // back substitute to compute the solution vector x.
        // Note that the columns of A correspond to rows of x.
        // Also note that A[rmax * 2 + 0] is equal to one.
        CPPAD_TESTVECTOR(AD<double>) x(2);
        x[1] = B[other] / A[other * 2 + 1];
        x[0] = B[rmax] - A[rmax * 2 + 1] * x[1];

        return x;
    }
}

bool vec_ad(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 4;
    CPPAD_TESTVECTOR(double)       x(n);
    CPPAD_TESTVECTOR(AD<double>) X(n);
    // 2 * identity matrix (rmax in Solve will be 0)
    X[0] = x[0] = 2.; X[1] = x[1] = 0.;
    X[2] = x[2] = 0.; X[3] = x[3] = 2.;

    // declare independent variables and start tape recording
    CppAD::Independent(X);

    // define the vector b
    CPPAD_TESTVECTOR(double)       b(2);
    CPPAD_TESTVECTOR(AD<double>) B(2);
    B[0] = b[0] = 0.;
    B[1] = b[1] = 1.;

    // range space vector solves X * Y = b
    size_t m = 2;
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y = Solve(X, B);

    // create f: X -> Y and stop tape recording
    CppAD::ADFun<double> f(X, Y);

    // By Cramer's rule:
    // y[0] = [ b[0] * x[3] - x[1] * b[1] ] / [ x[0] * x[3] - x[1] * x[2] ]
    // y[1] = [ x[0] * b[1] - b[0] * x[2] ] / [ x[0] * x[3] - x[1] * x[2] ]

    double den   = x[0] * x[3] - x[1] * x[2];
    double dsq   = den * den;
    double num0  = b[0] * x[3] - x[1] * b[1];
    double num1  = x[0] * b[1] - b[0] * x[2];

    // check value
    ok &= NearEqual(Y[0] , num0 / den, eps99, eps99);
    ok &= NearEqual(Y[1] , num1 / den, eps99, eps99);

    // forward computation of partials w.r.t. x[0]
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.; dx[1] = 0.;
    dx[2] = 0.; dx[3] = 0.;
    dy    = f.Forward(1, dx);
    ok &= NearEqual(dy[0], 0.         - num0 * x[3] / dsq, eps99, eps99);
    ok &= NearEqual(dy[1], b[1] / den - num1 * x[3] / dsq, eps99, eps99);

    // compute the solution for a new x matrix such that pivioting
    // on the original rmax row would divide by zero
    CPPAD_TESTVECTOR(double) y(m);
    x[0] = 0.; x[1] = 2.;
    x[2] = 2.; x[3] = 0.;

    // new values for Cramer's rule
    den   = x[0] * x[3] - x[1] * x[2];
    dsq   = den * den;
    num0  = b[0] * x[3] - x[1] * b[1];
    num1  = x[0] * b[1] - b[0] * x[2];

    // check values
    y    = f.Forward(0, x);
    ok &= NearEqual(y[0] , num0 / den, eps99, eps99);
    ok &= NearEqual(y[1] , num1 / den, eps99, eps99);

    // forward computation of partials w.r.t. x[1]
    dx[0] = 0.; dx[1] = 1.;
    dx[2] = 0.; dx[3] = 0.;
    dy    = f.Forward(1, dx);
    ok   &= NearEqual(dy[0],-b[1] / den + num0 * x[2] / dsq, eps99, eps99);
    ok   &= NearEqual(dy[1], 0.         + num1 * x[2] / dsq, eps99, eps99);

    // reverse computation of derivative of y[0] w.r.t x
    CPPAD_TESTVECTOR(double) w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0] = 1.; w[1] = 0.;
    dw   = f.Reverse(1, w);
    ok  &= NearEqual(dw[0], 0.         - num0 * x[3] / dsq, eps99, eps99);
    ok  &= NearEqual(dw[1],-b[1] / den + num0 * x[2] / dsq, eps99, eps99);
    ok  &= NearEqual(dw[2], 0.         + num0 * x[1] / dsq, eps99, eps99);
    ok  &= NearEqual(dw[3], b[0] / den - num0 * x[0] / dsq, eps99, eps99);

    return ok;
}

// END C++
