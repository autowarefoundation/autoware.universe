/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// modified version of test that used to be in ../example/print_for/print_for.cpp
# include <cppad/cppad.hpp>

namespace {
    using std::endl;
    using CppAD::AD;

    // use of PrintFor to check for invalid function arguments
    AD<double> check_log(const AD<double>& y)
    {   // check during recording
        CPPAD_ASSERT_UNKNOWN( y > 0. );

        // check during zero order forward calculation
        PrintFor(y, "check_log: y == ", y , " which is <= 0\n");

        return log(y);
    }
}

bool print_for(void)
{   bool ok = true;
    using CppAD::PrintFor;

    std::stringstream stream_out;   // stream that output is written to
    std::string       string_check; // what we expect the output to be

    // independent variable vector
    size_t n = 1;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 1.;
    Independent(ax);

    // print a VecAD<double>::reference object that is a parameter
    CppAD::VecAD<double> av(1);
    AD<double> Zero(0);
    av[Zero] = 0.;
    PrintFor("v[0] = ", av[Zero]);
    string_check += "v[0] = 0"; // v[0] == 0 during Forward(0, x)

    // Print a newline to separate this from previous output,
    // then print an AD<double> object that is a variable.
    PrintFor("\nv[0] + x[0] = ", av[0] + ax[0]);
    string_check += "\nv[0] + x[0] = 2"; // x[0] == 2 during Forward(0, x)

    // A conditional print that will not generate output when x[0] = 2.
    PrintFor(ax[0], "\n  2. + x[0] = ",   2. + ax[0], "\n");

    // A conditional print that will generate output when x[0] = 2.
    PrintFor(ax[0] - 2., "\n  3. + x[0] = ",   3. + ax[0], "\n");
    string_check += "\n  3. + x[0] = 5\n";

    // A log evaluations that will result in an error message when x[0] = 2.
    AD<double> var     = 2. - ax[0];
    AD<double> log_var = check_log(var);
    string_check += "check_log: y == 0 which is <= 0\n";

    // dependent variable vector
    size_t m = 2;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = av[Zero] + ax[0];

    // define f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // zero order forward with x[0] = 2
    CPPAD_TESTVECTOR(double) x(n);
    x[0] = 2.;
    f.Forward(0, x, stream_out);

    std::string string_out = stream_out.str();
    ok &= string_out == string_check;
    return ok;
}
