/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
Testing ipopt::solve
*/

// CPPAD_HAS_* defines
# include <cppad/configure.hpp>

# if CPPAD_HAS_IPOPT

# include <cppad/ipopt/solve.hpp>

namespace {
    using CppAD::AD;

    class FG_eval {
    public:
        typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
        void operator()(ADvector& fg, const ADvector& x)
        {   assert( fg.size() == 3 );
            assert( x.size()  == 4 );

            // Fortran style indexing
            AD<double> x1 = x[0];
            AD<double> x2 = x[1];
            AD<double> x3 = x[2];
            AD<double> x4 = x[3];
            // f(x)
            fg[0] = x1 * x4 * (x1 + x2 + x3) + x3;
            // g_1 (x)
            fg[1] = x1 * x2 * x3 * x4;
            // g_2 (x)
            fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
            //
            return;
        }
    };
}

bool ipopt_solve(void)
{   bool ok = true;
    size_t i, j;
    typedef CPPAD_TESTVECTOR( double ) Dvector;

    // number of independent variables (domain dimension for f and g)
    size_t nx = 4;
    // number of constraints (range dimension for g)
    size_t ng = 2;
    // initial value of the independent variables
    Dvector xi(nx);
    xi[0] = 1.0;
    xi[1] = 5.0;
    xi[2] = 5.0;
    xi[3] = 1.0;
    // lower and upper limits for x
    Dvector xl(nx), xu(nx);
    for(j = 0; j < nx; j++)
    {   xl[j] = 1.0;
        xu[j] = 5.0;
    }
    // lower and upper limits for g
    Dvector gl(ng), gu(ng);
    gl[0] = 25.0;     gu[0] = 1.0e19;
    gl[1] = 40.0;     gu[1] = 40.0;

    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    std::string base_options;
    // turn off any printing
    base_options += "Integer print_level  0\n";
    base_options += "String  sb         yes\n";
    // maximum number of iterations
    base_options += "Integer max_iter     10\n";
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    base_options += "Numeric tol          1e-6\n";
    // derivative testing
    base_options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    base_options += "Numeric point_perturbation_radius  0.\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solution values and tolerances
    double check_x[]  = { 1.000000, 4.743000, 3.82115, 1.379408 };
    double check_zl[] = { 1.087871, 0.,       0.,      0.       };
    double check_zu[] = { 0.,       0.,       0.,      0.       };
    double rel_tol    = 1e-6;  // relative tolerance
    double abs_tol    = 1e-6;  // absolute tolerance

    for(i = 0; i < 3; i++)
    {   std::string options( base_options );
        if( i == 1 )
            options += "Sparse true forward\n";
        if( i == 2 )
            options += "Sparse true reverse\n";

        // solve the problem
        CppAD::ipopt::solve<Dvector, FG_eval>(
            options, xi, xl, xu, gl, gu, fg_eval, solution
        );
        ok &= solution.status==CppAD::ipopt::solve_result<Dvector>::success;
        //
        // Check some of the solution values
        for(j = 0; j < nx; j++)
        {   ok &= CppAD::NearEqual(
                check_x[j],  solution.x[j],   rel_tol, abs_tol
            );
            ok &= CppAD::NearEqual(
                check_zl[j], solution.zl[j], rel_tol, abs_tol
            );
            ok &= CppAD::NearEqual(
                check_zu[j], solution.zu[j], rel_tol, abs_tol
            );
        }
    }

    return ok;
}

# endif
