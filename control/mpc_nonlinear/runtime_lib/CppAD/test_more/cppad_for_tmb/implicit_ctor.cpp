/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/cppad.hpp>

// implicit constructor from double
bool implicit_ctor(void)
{   using CppAD::AD;
    bool ok = true;
    //
    AD< AD<double> > x = 5.0;
    ok &= Value(x) == 5.0;
    //
    return ok;
}
