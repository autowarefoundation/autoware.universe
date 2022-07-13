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
$begin forward_dir.cpp$$
$spell
    Cpp
$$

$section Forward Mode: Example and Test of Multiple Directions$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <limits>
# include <cppad/cppad.hpp>
bool forward_dir(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * std::numeric_limits<double>::epsilon();
    size_t j;

    // domain space vector
    size_t n = 3;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.;
    ax[1] = 1.;
    ax[2] = 2.;

    // declare independent variables and starting recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = ax[0] * ax[1] * ax[2];

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // initially, the variable values during taping are stored in f
    ok &= f.size_order() == 1;

    // zero order Taylor coefficients
    CPPAD_TESTVECTOR(double) x0(n), y0;
    for(j = 0; j < n; j++)
        x0[j] = double(j+1);
    y0          = f.Forward(0, x0);
    ok         &= size_t( y0.size() ) == m;
    double y_0  = 1.*2.*3.;
    ok         &= NearEqual(y0[0], y_0, eps, eps);

    // first order Taylor coefficients
    size_t r = 2, ell;
    CPPAD_TESTVECTOR(double) x1(r*n), y1;
    for(ell = 0; ell < r; ell++)
    {   for(j = 0; j < n; j++)
            x1[ r * j + ell ] = double(j + 1 + ell);
    }
    y1  = f.Forward(1, r, x1);
    ok &= size_t( y1.size() ) == r*m;

    // secondorder Taylor coefficients
    CPPAD_TESTVECTOR(double) x2(r*n), y2;
    for(ell = 0; ell < r; ell++)
    {   for(j = 0; j < n; j++)
            x2[ r * j + ell ] = 0.0;
    }
    y2  = f.Forward(2, r, x2);
    ok &= size_t( y2.size() ) == r*m;
    //
    // Y_0 (t)     = F[X_0(t)]
    //             =  (1 + 1t)(2 + 2t)(3 + 3t)
    double y_1_0   = 1.*2.*3. + 2.*1.*3. + 3.*1.*2.;
    double y_2_0   = 1.*2.*3. + 2.*1.*3. + 3.*1.*2.;
    //
    // Y_1 (t)     = F[X_1(t)]
    //             =  (1 + 2t)(2 + 3t)(3 + 4t)
    double y_1_1   = 2.*2.*3. + 3.*1.*3. + 4.*1.*2.;
    double y_2_1   = 1.*3.*4. + 2.*2.*4. + 3.*2.*3.;
    //
    ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
    ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
    ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
    ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
    //
    // check number of orders
    ok   &= f.size_order() == 3;
    //
    // check number of directions
    ok   &= f.size_direction() == 2;
    //
    return ok;
}
// END C++
