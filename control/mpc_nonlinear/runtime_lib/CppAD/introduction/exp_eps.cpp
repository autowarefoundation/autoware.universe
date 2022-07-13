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
# include <cmath>             // for fabs function
# include "exp_eps.hpp"       // definition of exp_eps algorithm
bool exp_eps(void)
{   double x       = .5;
    double epsilon = .2;
    double check   = 1 + .5 + .125; // include 1 term less than epsilon
    bool   ok      = std::fabs( exp_eps(x, epsilon) - check ) <= 1e-10;
    return ok;
}
// END C++
