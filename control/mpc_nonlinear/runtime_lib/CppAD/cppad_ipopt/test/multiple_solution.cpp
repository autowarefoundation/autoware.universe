/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

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
f(x)    = x[1]; k=0, ell=0, I[0] = 0, J[0] = 1
g_0 (x) = x[0]; k=0, ell=1, I[0] = 1, J[0] = 0
g_1 (x) = x[1]; k=0, ell=2, I[0] = 2, J[0] = 1

minimize   f(x)
subject to -1 <= g_0(x)  <= 0
            0 <= g_1 (x) <= 1

The solution is x[1] = 0 and x[0] arbitrary.
*/

class FG_J_changes : public cppad_ipopt_fg_info
{
private:
    bool retape_;
public:
    // constructor
    FG_J_changes(bool retape_in)
    : retape_ (retape_in)
    { }
    size_t number_functions(void)
    {   return 1; }
    size_t domain_size(size_t k)
    {   size_t q = 1;
        assert(k == 0);
        return q;
    }
    size_t range_size(size_t k)
    {   size_t p = 1;
        assert(k == 0);
        return p;
    }
    size_t number_terms(size_t k)
    {   size_t L = 3;
        assert(k == 0);
        return L;
    }
    void index(size_t k, size_t ell, SizeVector&I, SizeVector& J)
    {   assert( I.size() >= 1 );
        assert( J.size() >= 1 );
        I[0] = ell;
        if( ell == 0 )
        {   J[0] = 1;
            return;
        }
        J[0] = ell - 1;
        return;
    }
    // retape function
    bool retape(size_t k)
    {   return retape_; }
    ADVector eval_r(size_t k, const ADVector&  u)
    {
        assert( u.size() == 1 );
        ADVector r(1);
        r[0] = u[0] ;
        return r;
    }
};
} // end empty namespace

bool multiple_solution(void)
{
    bool ok = true;
    // number of independent variables (domain dimension for f and g)
    size_t n = 2;
    // number of constraints (range dimension for g)
    size_t m = 2;
    // initial value of the independent variables
    NumberVector x_i(n);
    NumberVector x_l(n);
    NumberVector x_u(n);

    size_t i = 0;
    for(i = 0; i < n; i++)
    {   x_i[i] = 0.;
        x_l[i] = -1.0;
        x_u[i] = +1.0;
    }

    // lower and upper limits for g
    NumberVector g_l(m);
    NumberVector g_u(m);
    g_l[0] = -1; g_u[0] = 0.;
    g_l[1] = 0.; g_u[1] = 1.;

    // object for evaluating function
    bool retape = false;
    FG_J_changes my_fg_info(retape);
    cppad_ipopt_fg_info *fg_info = &my_fg_info;

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

    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    app->Options()->SetNumericValue("tol", 1e-9);
    app->Options()-> SetStringValue("derivative_test", "second-order");

    // Initialize the IpoptApplication and process the options
    Ipopt::ApplicationReturnStatus status = app->Initialize();
    ok    &= status == Ipopt::Solve_Succeeded;

    // Run the IpoptApplication
    status = app->OptimizeTNLP(cppad_nlp);
    ok    &= status == Ipopt::Solve_Succeeded;

    /*
     Check solution status
     */
    ok &= solution.status == cppad_ipopt_solution::success;
    ok &= CppAD::NearEqual(solution.x[1], 0., 1e-6, 1e-6);

    return ok;
}
