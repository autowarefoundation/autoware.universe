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
$begin ad_ctor.cpp$$
$spell
    Cpp
$$

$section AD Constructors: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$
$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool ad_ctor(void)
{   bool ok = true;   // initialize test result flag
    using CppAD::AD;  // so can use AD in place of CppAD::AD

    // default constructor
    AD<double> a;
    a = 0.;
    ok &= a == 0.;

    // constructor from base type
    AD<double> b(1.);
    ok &= b == 1.;

    // constructor from another type that converts to the base type
    AD<double> c(2);
    ok &= c == 2.;

    // constructor from AD<Base>
    AD<double> d(c);
    ok &= d == 2.;

    // constructor from a VecAD<Base> element
    CppAD::VecAD<double> v(1);
    v[0] = 3.;
    AD<double> e( v[0] );
    ok &= e == 3.;

    return ok;
}

// END C++
