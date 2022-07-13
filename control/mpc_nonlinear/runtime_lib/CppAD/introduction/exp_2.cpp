/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
// BEGIN C++
# include <cmath>           // define fabs function
# include "exp_2.hpp"       // definition of exp_2 algorithm
bool exp_2(void)
{   double x     = .5;
    double check = 1 + x + x * x / 2.;
    bool   ok    = std::fabs( exp_2(x) - check ) <= 1e-10;
    return ok;
}
// END C++
