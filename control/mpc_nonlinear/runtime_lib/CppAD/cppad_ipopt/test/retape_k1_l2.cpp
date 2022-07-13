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
/*
A test case were retaping is required, no constraints, and L[0] > 1.
*/
class FG_info : public cppad_ipopt_fg_info
{
public:
    // derived class part of constructor
    FG_info (void)
    { }
    // r_0 (u) = u if u > 1 and u^2 otherwise.
    ADVector eval_r(size_t k, const ADVector&  u)
    {   ADVector r(1);
        if( u[0] > 1 )
            r[0] = u[0];
        else
            r[0] = u[0] * u[0];
        return r;
    }
    // operation sequence depends on u
    bool retape(size_t k)
    {   return true; }
    // K = 1
    size_t number_functions(void)
    {   return 1; }
    // q[k] = 1
    size_t domain_size(size_t k)
    {   return 1; }
    // p[k] = 1
    size_t range_size(size_t k)
    {   return 1; }
    // L[k] = 2
    size_t number_terms(size_t k)
    {   return 2; }
    // I_{k,ell} = 0     // objective function index
    // J_{k,ell} = ell   // argument index
    void index(size_t k, size_t ell, SizeVector& I, SizeVector& J)
    {   I[0] = 0;
        J[0] = ell;
    }
};
} // end empty namespace

bool retape_k1_l2(void)
{   bool ok = true;
    size_t j;

    // number of independent variables (domain dimension for f and g)
    size_t n = 2;
    // no constraints (range dimension for g)
    size_t m = 0;
    // initial value of the independent variables
    NumberVector x_i(n);
    x_i[0] = 0.0;    // below break in eval_r definition
    x_i[1] = 2.0;    // above break in eval_r definition
    // lower and upper limits for x
    NumberVector x_l(n);
    NumberVector x_u(n);
    for(j = 0; j < n; j++)
    {   x_l[j] = -5.0;
        x_u[j] = +5.0;
    }
    // lower and upper limits for g
    NumberVector g_l;
    NumberVector g_u;

    // object in derived class
    FG_info my_fg_info;
    cppad_ipopt_fg_info *fg_info = &my_fg_info;

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
    app->Options()-> SetStringValue("derivative_test", "second-order");
    app->Options()-> SetNumericValue("point_perturbation_radius", 0.);

    // Initialize the IpoptApplication and process the options
    Ipopt::ApplicationReturnStatus status = app->Initialize();
    ok    &= status == Ipopt::Solve_Succeeded;

    // Run the IpoptApplication
    status = app->OptimizeTNLP(cppad_nlp);
    ok    &= status == Ipopt::Solve_Succeeded;

    /*
    Check the solution values
    */
    ok &= solution.status == cppad_ipopt_solution::success;
    //
    double rel_tol     = 1e-6;  // relative tolerance
    double abs_tol     = 1e-6;  // absolute tolerance
    for(j = 0; j < n; j++)
        ok &= CppAD::NearEqual( 0., solution.x[j], rel_tol, abs_tol);

    return ok;
}
