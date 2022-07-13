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
Check the value of machine epsilon is accurate enough for the correctness tests
*/

# include <cfloat>
# include <limits>

bool dbl_epsilon(void)
{   bool ok = true;

    // CppAD correctness tests assume machine epsilon is less than 1e-13
    ok &= DBL_EPSILON < 1e-13;
    ok &= std::numeric_limits<double>::digits10 >= 13;

    return ok;
}
