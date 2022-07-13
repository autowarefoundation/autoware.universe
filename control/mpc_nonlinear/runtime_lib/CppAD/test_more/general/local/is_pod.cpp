/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cstddef>
# include <cppad/cppad.hpp>

bool is_pod(void)
{   bool ok = true;
    using CppAD::local::is_pod;
    //
    // Check all the cases that are the same as short int on
    // https://en.cppreference.com/w/cpp/language/types on 2020-12-04.
    ok &= is_pod<short>();
    ok &= is_pod<short int>();
    ok &= is_pod<signed short>();
    ok &= is_pod<signed short int>();
    //
    return ok;
}
