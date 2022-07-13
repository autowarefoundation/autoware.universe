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
$begin con_dyn_var.cpp$$

$section AD Parameter and Variable Functions: Example and Test$$




$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool con_dyn_var(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::VecAD;
    using CppAD::Parameter;
    using CppAD::Variable;

    // No recording active
    CPPAD_TESTVECTOR(AD<double>) x(1), y(1), dynamic(1);
    x[0]        = 0.0;
    y[0]        = 1.0;
    dynamic[0]  = 2.0;
    //
    ok &= Constant(x[0]);
    ok &= Parameter(x[0]);
    ok &= ! Dynamic(x[0]);
    ok &= ! Variable(x[0]);
    //
    ok &= Constant(y[0]);
    ok &= Parameter(y[0]);
    ok &= ! Dynamic(y[0]);
    ok &= ! Variable(y[0]);
    //
    ok &= Constant(dynamic[0]);
    ok &= Parameter(dynamic[0]);
    ok &= ! Dynamic(dynamic[0]);
    ok &= ! Variable(dynamic[0]);

    // declare independent variables and start recording
    CppAD::Independent(x, dynamic);
    //
    ok &= Variable(x[0]);
    ok &= ! Constant(x[0]);
    ok &= ! Dynamic(x[0]);
    ok &= ! Parameter(x[0]);
    //
    ok &= Constant(y[0]);
    ok &= Parameter(y[0]);
    ok &= ! Dynamic(y[0]);
    ok &= ! Variable(y[0]);
    //
    ok &= Dynamic(dynamic[0]);
    ok &= Parameter(dynamic[0]);
    ok &= ! Constant(dynamic[0]);
    ok &= ! Variable(dynamic[0]);

    // a dependent variable
    y[0] = fabs(x[0]) * dynamic[0];
    ok  &= Variable(y[0]);
    ok  &= ! Constant(y[0]);
    ok  &= ! Dynamic(y[0]);
    ok  &= ! Parameter(y[0]);

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);
    //
    ok &= Constant(x[0]);
    ok &= Parameter(x[0]);
    ok &= ! Dynamic(x[0]);
    ok &= ! Variable(x[0]);
    //
    ok &= Constant(y[0]);
    ok &= Parameter(y[0]);
    ok &= ! Dynamic(y[0]);
    ok &= ! Variable(y[0]);
    //
    ok &= Constant(dynamic[0]);
    ok &= Parameter(dynamic[0]);
    ok &= ! Dynamic(dynamic[0]);
    ok &= ! Variable(dynamic[0]);

    return ok;
}

// END C++
