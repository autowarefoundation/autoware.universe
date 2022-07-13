/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin atan2.cpp$$
$spell
    tan
    atan
$$

$section The AD atan2 Function: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# define N_THETA 20

bool atan2(void)
{   bool ok = true;
    //
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    double pi    = 2.0 * std::atan(1.0);
    //
    for(size_t k = 0; k < N_THETA; ++k)
    {   // theta
        double theta =  2.0 * pi * double(k+1) / double(N_THETA) - pi;
        //
        // radius
        double radius = 1.0 + double(k) / double(N_THETA);
        //
        // x, y
        double x = radius * std::cos(theta);
        double y = radius * std::sin(theta);
        //
        // au
        CPPAD_TESTVECTOR(AD<double>) au(2);
        au[0] = x;
        au[1] = y;
        CppAD::Independent(au);
        //
        // av
        CPPAD_TESTVECTOR(AD<double>) av(1);
        av[0] = CppAD::atan2(au[1], au[0]);
        //
        // f(x, y) = atan2(y, x)
        CppAD::ADFun<double> f(au, av);
        //
        // check value
        ok &= NearEqual(av[0] , theta, eps99, eps99);
        //
        // partial_x, partial_y
        // see https://en.wikipedia.org/wiki/Atan2#Derivative
        double partial_x = - y / (radius * radius);
        double partial_y =   x / (radius * radius);
        //
        // check forward mode
        CPPAD_TESTVECTOR(double) du(2), dv(1);
        du[0] = 1.0;
        du[1] = 0.0;
        dv    = f.Forward(1, du);
        ok   &= NearEqual(dv[0], partial_x, eps99, eps99);
        du[0] = 0.0;
        du[1] = 1.0;
        dv    = f.Forward(1, du);
        ok   &= NearEqual(dv[0], partial_y, eps99, eps99);
        //
        // check reverse mode
        CPPAD_TESTVECTOR(double)  w(1);
        CPPAD_TESTVECTOR(double) dw(2);
        w[0]  = 1.;
        dw    = f.Reverse(1, w);
        ok   &= NearEqual(dw[0], partial_x, eps99, eps99);
        ok   &= NearEqual(dw[1], partial_y, eps99, eps99);
        //
    }
    return ok;
}

// END C++
