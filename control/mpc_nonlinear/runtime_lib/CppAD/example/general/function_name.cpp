/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin function_name.cpp$$
$spell
$$

$section ADFun Function Name: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool function_name(void)
{   bool ok = true;
    using CppAD::AD;

    // create empty function
    CppAD::ADFun<double> f;

    // check its name
    ok &= f.function_name_get() == "";

    // set and check a new name
    f.function_name_set("empty_function");
    ok &= f.function_name_get() == "empty_function";

    // store an operation sequence in f
    size_t nx = 1;
    size_t ny = 1;
    CPPAD_TESTVECTOR( AD<double> ) ax(nx), ay(ny);
    ax[0] = 1.0;
    CppAD::Independent(ax);
    ay[0] = sin(ax[0]);
    f.Dependent(ax, ay);

    // check fucntion name has not changed
    ok &= f.function_name_get() == "empty_function";

    // now set a better name for this function
    f.function_name_set("sin");
    ok &= f.function_name_get() == "sin";

    return ok;
}

// END C++
