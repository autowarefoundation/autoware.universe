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
$begin near_equal.cpp$$
$spell
    cpp
    abs
o   Microsoft
$$

$section NearEqual Function: Example and Test$$

$head File Name$$
This file is called $code near_equal.cpp$$ instead of
$code NearEqual.cpp$$
to avoid a name conflict with $code ../lib/NearEqual.cpp$$
in the corresponding Microsoft project file.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/utility/near_equal.hpp>

# include <complex>

bool Near_Equal(void)
{   bool ok = true;
    typedef std::complex<double> Complex;
    using CppAD::NearEqual;

    // double
    double x    = 1.00000;
    double y    = 1.00001;
    double a    =  .00003;
    double r    =  .00003;
    double zero = 0.;
    double inf  = 1. / zero;
    double nan  = 0. / zero;

    ok &= NearEqual(x, y, zero, a);
    ok &= NearEqual(x, y, r, zero);
    ok &= NearEqual(x, y, r, a);

    ok &= ! NearEqual(x, y, r / 10., a / 10.);
    ok &= ! NearEqual(inf, inf, r, a);
    ok &= ! NearEqual(-inf, -inf, r, a);
    ok &= ! NearEqual(nan, nan, r, a);

    // complex
    Complex X(x, x / 2.);
    Complex Y(y, y / 2.);
    Complex Inf(inf, zero);
    Complex Nan(zero, nan);

    ok &= NearEqual(X, Y, zero, a);
    ok &= NearEqual(X, Y, r, zero);
    ok &= NearEqual(X, Y, r, a);

    ok &= ! NearEqual(X, Y, r / 10., a / 10.);
    ok &= ! NearEqual(Inf, Inf, r, a);
    ok &= ! NearEqual(-Inf, -inf, r, a);
    ok &= ! NearEqual(Nan, Nan, r, a);

    return ok;
}

// END C++
