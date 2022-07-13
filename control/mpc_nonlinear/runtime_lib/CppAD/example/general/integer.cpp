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
$begin integer.cpp$$
$spell
    Cpp
    cstddef
$$

$section Convert From AD to Integer: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool Integer(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::Integer;

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0] = 3.5;
    x[1] = 4.5;

    // check integer before recording
    ok &= (Integer(x[0]) == 3);
    ok &= (Integer(x[1]) == 4);

    // start recording

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // check integer during recording
    ok &= (Integer(x[0]) == 3);
    ok &= (Integer(x[1]) == 4);

    // check integer for VecAD element
    CppAD::VecAD<double> v(1);
    AD<double> zero(0);
    v[zero] = 2;
    ok &= (Integer(v[zero]) == 2);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) y(m);
    y[0] = - x[1];

    // create f: x -> y and stop recording
    CppAD::ADFun<double> f(x, y);

    // check integer after recording
    ok &= (Integer(x[0]) ==  3.);
    ok &= (Integer(x[1]) ==  4.);
    ok &= (Integer(y[0]) == -4.);

    return ok;
}
// END C++
