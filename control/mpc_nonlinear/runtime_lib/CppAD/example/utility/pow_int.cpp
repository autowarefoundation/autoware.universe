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
$begin pow_int.cpp$$
$spell
    tan
    atan
$$

$section The Pow Integer Exponent: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>

bool pow_int(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // declare independent variables and start tape recording
    size_t n  = 1;
    double x0 = -0.5;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]      = x0;
    CppAD::Independent(x);

    // dependent variable vector
    size_t m = 7;
    CPPAD_TESTVECTOR(AD<double>) y(m);
    for(size_t i = 0; i < m; i++)
        y[i] = CppAD::pow(x[0], int(i) - 3);

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);

    // check value
    double check;
    for(size_t i = 0; i < m; i++)
    {   check = std::pow(x0, double(i) - 3.0);
        ok &= NearEqual(y[i] , check,  eps99 , eps99);
    }

    // forward computation of first partial w.r.t. x[0]
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dy    = f.Forward(1, dx);
    for(size_t i = 0; i < m; i++)
    {   check = (double(i) - 3.0) * std::pow(x0, double(i) - 4.0);
        ok &= NearEqual(dy[i] , check,  eps99 , eps99);
    }

    // reverse computation of derivative of y[i]
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    for(size_t i = 0; i < m; i++)
        w[i] = 0.;
    for(size_t i = 0; i < m; i++)
    {   w[i] = 1.;
        dw    = f.Reverse(1, w);
        check = (double(i) - 3.0) * std::pow(x0, double(i) - 4.0);
        ok &= NearEqual(dw[0] , check,  eps99 , eps99);
        w[i] = 0.;
    }

    return ok;
}

// END C++
