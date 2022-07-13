/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad_ipopt_nlp.hpp>

namespace { // Begin empty namespace
using namespace cppad_ipopt;

// ---------------------------------------------------------------------------
class FG_retape : public cppad_ipopt_fg_info
{
public:
    // derived class part of constructor
    FG_retape(void)
    { }
    // Evaluation of the objective f(x), and constraints g(x)
    // using an Algorithmic Differentiation (AD) class.
    ADVector eval_r(size_t k, const ADVector&  x)
    {   ADVector fg(2);

        // f(x)
        if( x[0] >= 1. )
            fg[0] = .5 * (x[0] * x[0] + x[1] * x[1]);
        else
            fg[0] = x[0] + .5 * x[1] * x[1];
        // g (x)
        fg[1] = x[0];

        return fg;
    }
    bool retape(size_t k)
    {   return true; }
};
} // end of empty namespace

bool retape_k1_l1(void)
{   bool ok = true;
    size_t j;


    // number of independent variables (domain dimension for f and g)
    size_t n = 2;
    // number of constraints (range dimension for g)
    size_t m = 1;
    // initial value of the independent variables
    NumberVector x_i(n);
    x_i[0] = 2.0;
    x_i[1] = 2.0;
    // lower and upper limits for x
    NumberVector x_l(n);
    NumberVector x_u(n);
    for(j = 0; j < n; j++)
    {   x_l[j] = -10.; x_u[j] = +10.;
    }
    // lower and upper limits for g
    NumberVector g_l(m);
    NumberVector g_u(m);
    g_l[0] = -1.;     g_u[0] = 1.0e19;

    // object in derived class
    FG_retape fg_retape;
    cppad_ipopt_fg_info *fg_info = &fg_retape;

    // create the Ipopt interface
    cppad_ipopt_solution solution;
    Ipopt::SmartPtr<Ipopt::TNLP> cppad_nlp = new cppad_ipopt_nlp(
        n, m, x_i, x_l, x_u, g_l, g_u, fg_info, &solution
    );

    // Create an instance of the IpoptApplication
    using Ipopt::IpoptApplication;
    Ipopt::SmartPtr<IpoptApplication> app = new IpoptApplication();

    // turn off any printing
    app->Options()->SetIntegerValue("print_level", 0);
    app->Options()->SetStringValue("sb", "yes");

    // maximum number of iterations
    app->Options()->SetIntegerValue("max_iter", 10);

    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    app->Options()->SetNumericValue("tol", 1e-9);

    // derivative testing
    app->Options()->
    SetStringValue("derivative_test", "second-order");

    // Initialize the IpoptApplication and process the options
    Ipopt::ApplicationReturnStatus status = app->Initialize();
    ok    &= status == Ipopt::Solve_Succeeded;

    // Run the IpoptApplication
    status = app->OptimizeTNLP(cppad_nlp);
    ok    &= status == Ipopt::Solve_Succeeded;

    /*
    Check some of the solution values
    */
    ok &= solution.status == cppad_ipopt_solution::success;
    //
    double check_x[]   = { -1., 0. };
    double rel_tol     = 1e-6;  // relative tolerance
    double abs_tol     = 1e-6;  // absolute tolerance
    for(j = 0; j < n; j++)
    {   ok &= CppAD::NearEqual(
            check_x[j],   solution.x[j],   rel_tol, abs_tol
        );
    }

    return ok;
}
