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
$begin var2par.cpp$$
$spell
    Var
    Cpp
$$

$section Convert a Variable or Dynamic Parameter a Constant: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>


bool Var2Par(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::Value;
    using CppAD::Var2Par;

    // independent variables
    size_t nx = 2;
    CPPAD_TESTVECTOR(AD<double>) ax(nx);
    ax[0] = 3.;
    ax[1] = 4.;

    // independent dynamic paramers
    size_t np = 1;
    CPPAD_TESTVECTOR(AD<double>) ap(np);
    ap[0] = 5.;

    // declare independent variables and dynamic parameters
    CppAD::Independent(ax, ap);

    // range space vector
    size_t ny = 2;
    CPPAD_TESTVECTOR(AD<double>) ay(ny);
    ay[0] = - ax[1] * Var2Par(ax[0]);    // same as ay[0] = -ax[1] * 3.;
    ay[1] = - ax[1] * Var2Par(ap[0]);    // same as ay[1] = -ax[1] * 5.;

    // Must convert these objects to constants before calling Value
    ok &= ( Value( Var2Par(ax[0]) ) == 3. );
    ok &= ( Value( Var2Par(ax[1]) ) == 4. );
    ok &= ( Value( Var2Par(ap[0]) ) == 5. );
    ok &= ( Value( Var2Par(ay[0]) ) == -12. );
    ok &= ( Value( Var2Par(ay[1]) ) == -20. );

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // All AD object are currently constants
    ok &= (Value(ax[0]) ==  3.);
    ok &= (Value(ax[1]) ==  4.);
    ok &= (Value(ap[0]) ==  5.);
    ok &= (Value(ay[0]) == -12.);
    ok &= (Value(ay[1]) == -20.);

    // evaluate zero order forward mode
    // (note that the only real variable in this recording is x[1])
    CPPAD_TESTVECTOR(double) x(nx), p(np), y(ny);
    x[0] = 6.;
    x[1] = 7.;
    p[0] = 8.;
    f.new_dynamic(p);
    y    = f.Forward(0, x);
    ok  &= y[0] == - x[1] * 3.0;
    ok  &= y[1] == - x[1] * 5.0;

    return ok;
}
// END C++
