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
$begin pow.cpp$$
$spell
$$

$section The AD Power Function: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>

bool pow(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 2;
    double x = 0.5;
    double y = 2.;
    CPPAD_TESTVECTOR(AD<double>) axy(n);
    axy[0]      = x;
    axy[1]      = y;

    // declare independent variables and start tape recording
    CppAD::Independent(axy);

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>) az(m);
    az[0] = CppAD::pow(axy[0], axy[1]); // pow(variable, variable)
    az[1] = CppAD::pow(axy[0], y);      // pow(variable, parameter)
    az[2] = CppAD::pow(x,     axy[1]);  // pow(parameter, variable)

    // create f: axy -> az and stop tape recording
    CppAD::ADFun<double> f(axy, az);

    // check value
    double check = std::pow(x, y);
    size_t i;
    for(i = 0; i < m; i++)
        ok &= NearEqual(az[i] , check,  eps, eps);

    // forward computation of first partial w.r.t. x
    CPPAD_TESTVECTOR(double) dxy(n);
    CPPAD_TESTVECTOR(double) dz(m);
    dxy[0] = 1.;
    dxy[1] = 0.;
    dz    = f.Forward(1, dxy);
    check = y * std::pow(x, y-1.);
    ok   &= NearEqual(dz[0], check, eps, eps);
    ok   &= NearEqual(dz[1], check, eps, eps);
    ok   &= NearEqual(dz[2],    0., eps, eps);

    // forward computation of first partial w.r.t. y
    dxy[0] = 0.;
    dxy[1] = 1.;
    dz    = f.Forward(1, dxy);
    check = std::log(x) * std::pow(x, y);
    ok   &= NearEqual(dz[0], check, eps, eps);
    ok   &= NearEqual(dz[1],    0., eps, eps);
    ok   &= NearEqual(dz[2], check, eps, eps);

    // reverse computation of derivative of z[0] + z[1] + z[2]
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0]  = 1.;
    w[1]  = 1.;
    w[2]  = 1.;
    dw    = f.Reverse(1, w);
    check = y * std::pow(x, y-1.);
    ok   &= NearEqual(dw[0], 2. * check, eps, eps);
    check = std::log(x) * std::pow(x, y);
    ok   &= NearEqual(dw[1], 2. * check, eps, eps);

    // use a VecAD<Base>::reference object with pow
    CppAD::VecAD<double> v(2);
    AD<double> zero(0);
    AD<double> one(1);
    v[zero]           = axy[0];
    v[one]            = axy[1];
    AD<double> result = CppAD::pow(v[zero], v[one]);
    ok               &= NearEqual(result, az[0], eps, eps);

    return ok;
}

// END C++
