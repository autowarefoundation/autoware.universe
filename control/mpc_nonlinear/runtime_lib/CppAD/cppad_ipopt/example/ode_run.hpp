# ifndef CPPAD_CPPAD_IPOPT_EXAMPLE_ODE_RUN_HPP
# define CPPAD_CPPAD_IPOPT_EXAMPLE_ODE_RUN_HPP
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
$begin ipopt_nlp_ode_run.hpp$$
$spell
    Ipopt
$$

$section Driver for Running the Ipopt ODE Example$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++
# include "ode_problem.hpp"

namespace { // BEGIN empty namespace -----------------------------------------
using namespace cppad_ipopt;

template <class FG_info>
void ipopt_ode_case(
    bool  retape        ,
    const SizeVector& N ,
    NumberVector&     x )
{
    size_t i, j;

    // compute the partial sums of the number of grid points
    assert( N.size() == Nz + 1);
    assert( N[0] == 0 );
    SizeVector S(Nz+1);
    S[0] = 0;
    for(i = 1; i <= Nz; i++)
        S[i] = S[i-1] + N[i];

    // number of components of x corresponding to values for y
    size_t ny_inx = (S[Nz] + 1) * Ny;
    // number of constraints (range dimension of g)
    size_t m      = ny_inx;
    // number of components in x (domain dimension for f and g)
    size_t n      = ny_inx + Na;
    // the argument vector for the optimization is
    // y(t) at t[0] , ... , t[S[Nz]] , followed by a
    NumberVector x_i(n), x_l(n), x_u(n);
    for(j = 0; j < ny_inx; j++)
    {   x_i[j] = 0.;       // initial y(t) for optimization
        x_l[j] = -1.0e19;  // no lower limit
        x_u[j] = +1.0e19;  // no upper limit
    }
    for(j = 0; j < Na; j++)
    {   x_i[ny_inx + j ] = .5;       // initiali a for optimization
        x_l[ny_inx + j ] =  -1.e19;  // no lower limit
        x_u[ny_inx + j ] =  +1.e19;  // no upper
    }
    // all of the difference equations are constrained to the value zero
    NumberVector g_l(m), g_u(m);
    for(i = 0; i < m; i++)
    {   g_l[i] = 0.;
        g_u[i] = 0.;
    }

    // object defining the objective f(x) and constraints g(x)
    FG_info fg_info(retape, N);

    // create the CppAD Ipopt interface
    cppad_ipopt_solution solution;
    Ipopt::SmartPtr<Ipopt::TNLP> cppad_nlp = new cppad_ipopt_nlp(
        n, m, x_i, x_l, x_u, g_l, g_u, &fg_info, &solution
    );

    // Create an Ipopt application
    using Ipopt::IpoptApplication;
    Ipopt::SmartPtr<IpoptApplication> app = new IpoptApplication();

    // turn off any printing
    app->Options()->SetIntegerValue("print_level", 0);
    app->Options()->SetStringValue("sb", "yes");

    // maximum number of iterations
    app->Options()->SetIntegerValue("max_iter", 30);

    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    app->Options()->SetNumericValue("tol", 1e-9);

    // Derivative testing is very slow for large problems
    // so comment this out if you use a large value for N[].
    app->Options()-> SetStringValue( "derivative_test", "second-order");
    app->Options()-> SetNumericValue( "point_perturbation_radius", 0.);

# ifndef NDEBUG
    bool ok = true;
    //
    // Initialize the application and process the options
    Ipopt::ApplicationReturnStatus status = app->Initialize();
    ok    &= status == Ipopt::Solve_Succeeded;
    //
    // Run the application
    status = app->OptimizeTNLP(cppad_nlp);
    ok    &= status == Ipopt::Solve_Succeeded;
    //
    assert(ok);
# else
    app->Initialize();
    app->OptimizeTNLP(cppad_nlp);
# endif

    // return the solution
    x.resize( solution.x.size() );
    for(j = 0; j < x.size(); j++)
        x[j] = solution.x[j];

    return;
}
} // END empty namespace ----------------------------------------------------
// END C++

# endif
