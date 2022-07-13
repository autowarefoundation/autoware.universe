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
$begin nan.cpp$$

$section nan: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++
# include <cppad/utility/nan.hpp>
# include <vector>
# include <limits>

bool nan(void)
{   bool ok = true;

    // get a nan
    double double_zero = 0.;
    double double_nan = std::numeric_limits<double>::quiet_NaN();

    // create a simple vector with no nans
    std::vector<double> v(2);
    v[0] = double_zero;
    v[1] = double_zero;

    // check that zero is not nan
    ok &= ! CppAD::isnan(double_zero);
    ok &= ! CppAD::hasnan(v);

    // check that nan is a nan
    v[1] = double_nan;
    ok &= CppAD::isnan(double_nan);
    ok &= CppAD::hasnan(v);

    // check that nan is not equal to itself
    ok &= (double_nan != double_nan);

    return ok;
}

// END C++
