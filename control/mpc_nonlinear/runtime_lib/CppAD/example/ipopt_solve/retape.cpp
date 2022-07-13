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

$begin ipopt_solve_retape.cpp$$
$spell
    Retaping
    retape
    cppad_nlp
    IpoptDir
    CppAD
$$

$section Nonlinear Programming Retaping: Example and Test$$

$head Purpose$$
This example program demonstrates a case were the $code ipopt::solve$$
argument $cref/retape/ipopt_solve/options/Retape/$$ should be true.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/ipopt/solve.hpp>

namespace {
    using CppAD::AD;

    class FG_eval {
    public:
        typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
        void operator()(ADvector& fg, const ADvector& x)
        {   assert( fg.size() == 1 );
            assert( x.size()  == 1 );

            // compute the Huber function using a conditional
            // statement that depends on the value of x.
            double eps = 0.1;
            if( fabs(x[0]) <= eps )
                fg[0] = x[0] * x[0] / (2.0 * eps);
            else
                fg[0] = fabs(x[0]) - eps / 2.0;

            return;
        }
    };
}

bool retape(void)
{   bool ok = true;
    typedef CPPAD_TESTVECTOR( double ) Dvector;

    // number of independent variables (domain dimension for f and g)
    size_t nx = 1;
    // number of constraints (range dimension for g)
    size_t ng = 0;
    // initial value, lower and upper limits, for the independent variables
    Dvector xi(nx), xl(nx), xu(nx);
    xi[0] = 2.0;
    xl[0] = -1e+19;
    xu[0] = +1e+19;
    // lower and upper limits for g
    Dvector gl(ng), gu(ng);

    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    std::string options;
    // retape operation sequence for each new x
    options += "Retape  true\n";
    // turn off any printing
    options += "Integer print_level   0\n";
    options += "String  sb          yes\n";
    // maximum number of iterations
    options += "Integer max_iter      10\n";
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol           1e-9\n";
    // derivative testing
    options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius  0.\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution
    );
    //
    // Check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    double rel_tol    = 1e-6;  // relative tolerance
    double abs_tol    = 1e-6;  // absolute tolerance
    ok &= CppAD::NearEqual( solution.x[0], 0.0,  rel_tol, abs_tol);

    return ok;
}
// END C++
