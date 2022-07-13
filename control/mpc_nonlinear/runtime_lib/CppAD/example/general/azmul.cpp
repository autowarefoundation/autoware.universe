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
$begin azmul.cpp$$
$spell
$$

$section AD Absolute Zero Multiplication: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>

bool azmul(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double inf = std::numeric_limits<double>::infinity();
    double eps = 10. * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 2;
    double x = 0.5;
    double y = 2.0;
    CPPAD_TESTVECTOR(AD<double>) axy(n);
    axy[0]      = x;
    axy[1]      = y;

    // declare independent variables and start tape recording
    CppAD::Independent(axy);

    // range space vector
    size_t m = 5;
    CPPAD_TESTVECTOR(AD<double>) az(m);
    az[0] = CppAD::azmul(axy[0], axy[1]); // azmul(variable, variable)
    az[1] = CppAD::azmul(axy[0], inf);    // azmul(variable, parameter=inf)
    az[2] = CppAD::azmul(axy[0], 3.0);    // azmul(variable, parameter=3.0)
    az[3] = CppAD::azmul(0.0, axy[1]);    // azmul(parameter=0.0, variable)
    az[4] = CppAD::azmul(4.0, axy[1]);    // azmul(parameter=4.0, variable)

    // create f: axy -> az and stop tape recording
    CppAD::ADFun<double> f(axy, az);

    // check value when x is not zero
    ok &= NearEqual(az[0] , x * y,  eps, eps);
    ok &= az[1] == inf;
    ok &= NearEqual(az[2] , x * 3.0,  eps, eps);
    ok &= az[3] == 0.0;
    ok &= NearEqual(az[4] , 4.0 * y,  eps, eps);


    // check value x is zero and y is infinity
    CPPAD_TESTVECTOR(double) xy(n), z(m);
    xy[0] = 0.0;
    xy[1] = inf;
    z     = f.Forward(0, xy);
    ok &= z[0] == 0.0;
    ok &= z[1] == 0.0;
    ok &= z[2] == 0.0;
    ok &= z[3] == 0.0;
    ok &= z[4] == inf;

    return ok;
}

// END C++
