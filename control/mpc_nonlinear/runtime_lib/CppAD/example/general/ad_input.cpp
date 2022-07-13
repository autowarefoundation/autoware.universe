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
$begin ad_input.cpp$$
$spell
    Cpp
    cstddef
$$

$section AD Output Operator: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

# include <sstream>  // std::istringstream
# include <string>   // std::string

bool ad_input(void)
{   bool ok = true;

    // create the input string stream is.
    std::string str ("123 456");
    std::istringstream is(str);

    // start and AD<double> recording
    CPPAD_TESTVECTOR( CppAD::AD<double> ) x(1), y(1);
    x[0] = 1.0;
    CppAD::Independent(x);
    CppAD::AD<double> z = x[0];
    ok &= Variable(z);

    // read first number into z and second into y[0]
    is >> z >> y[0];
    ok   &= Parameter(z);
    ok   &= (z == 123.);
    ok   &= Parameter(y[0]);
    ok   &= (y[0] == 456.);
    //
    // terminate recording starting by call to Independent
    CppAD::ADFun<double> f(x, y);

    return ok;
}
// END C++
