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
$begin ad_assign.cpp$$
$spell
    Cpp
$$

$section AD Assignment: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$
$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool ad_assign(void)
{   bool ok = true;   // initialize test result flag
    using CppAD::AD;  // so can use AD in place of CppAD::AD

    // assignment to base value
    AD<double> a;
    a = 1.;
    ok &= a == 1.;

    // assignment to a value that converts to the base type
    a = 2;
    ok &= a == 2.;

    // assignment to an AD<Base>
    AD<double> b(3.);
    a = b;
    ok &= a == 3.;

    // assignment to an VecAD<Base> element
    CppAD::VecAD<double> v(1);
    v[0] = 4.;
    a = v[0];
    ok &= a == 4.;

    return ok;
}
// END C++
