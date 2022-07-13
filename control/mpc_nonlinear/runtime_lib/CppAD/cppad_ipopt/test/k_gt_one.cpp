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
This solve the same problem as
../cppad_ipopt/cppad_ipopt_simple.cpp (repository revision
1276) in a convoluted way in order to test the representation code.
*/
class FG_K_gt_one : public cppad_ipopt_fg_info
{
private:
    bool retape_;
public:
    // derived class part of constructor
    FG_K_gt_one(bool retape_in)
    : retape_ (retape_in)
    { }
    // Evaluation of the objective f(x), and constraints g(x)
    // using an Algorithmic Differentiation (AD) class.
    ADVector eval_r(size_t k, const ADVector&  u)
    {

        // Fortran style indexing
        ADNumber x1 = u[3];
        ADNumber x2 = u[2];
        ADNumber x3 = u[1];
        ADNumber x4 = u[0];
        if( k == 0 )
        {   ADVector r(1);
            // f(x)
            r[0] = x1 * x4 * (x1 + x2 + x3) + x3;
            return r;
        }
        ADVector r(2);
        // g_1 (x)
        r[0] = x1 * x2 * x3 * x4;
        // g_2 (x)
        r[1] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;
        return r;
    }
    bool retape(size_t k)
    {   return retape_; }
    size_t number_functions(void)
    {   return 2; }
    size_t domain_size(size_t k)
    {   return 4; }
    size_t range_size(size_t k)
    {   if( k == 0 )
            return 1;
        return 2;
    }
    size_t number_terms(size_t k)
    {   return 1; }
    void index(size_t k, size_t ell, SizeVector& I, SizeVector& J)
    {
        if( k == 0 )
            I[0] = 0;
        else
        {   I[0] = 1;
            I[1] = 2;
        }
        // reverse the order of the variables in u from that in x
        for(size_t j = 0; j < 4; j++)
            J[j] = 3-j;
    }
};
} // end empty namespace

bool k_gt_one(void)
{   bool ok = true;
    size_t j;


    // number of independent variables (domain dimension for f and g)
    size_t n = 4;
    // number of constraints (range dimension for g)
    size_t m = 2;
    // initial value of the independent variables
    NumberVector x_i(n);
    x_i[0] = 1.0;
    x_i[1] = 5.0;
    x_i[2] = 5.0;
    x_i[3] = 1.0;
    // lower and upper limits for x
    NumberVector x_l(n);
    NumberVector x_u(n);
    for(j = 0; j < n; j++)
    {   x_l[j] = 1.0;
        x_u[j] = 5.0;
    }
    // lower and upper limits for g
    NumberVector g_l(m);
    NumberVector g_u(m);
    g_l[0] = 25.0;     g_u[0] = 1.0e19;
    g_l[1] = 40.0;     g_u[1] = 40.0;

    // known solution to check against
    double check_x[]   = { 1.000000, 4.743000, 3.82115, 1.379408 };

    size_t icase;
    for(icase = 0; icase <= 1; icase++)
    {   // Should cppad_ipopt_nlp retape the operation sequence for
        // every new x. Can test both true and false cases because
        // the operation sequence does not depend on x (for this case).
        bool retape = bool(icase);

        // check case where upper and lower limits are equal
        if( icase == 1 )
        {   x_l[2] = check_x[2];
            x_u[2] = check_x[2];
        }

        // object in derived class
        FG_K_gt_one my_fg_info(retape);
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
        double check_z_l[] = { 1.087871, 0.,       0.,      0.       };
        double check_z_u[] = { 0.,       0.,       0.,      0.       };
        double rel_tol     = 1e-6;  // relative tolerance
        double abs_tol     = 1e-6;  // absolute tolerance
        for(j = 0; j < n; j++)
        {   ok &= CppAD::NearEqual(
            check_x[j],   solution.x[j],   rel_tol, abs_tol
            );
            ok &= CppAD::NearEqual(
            check_z_l[j], solution.z_l[j], rel_tol, abs_tol
            );
            ok &= CppAD::NearEqual(
            check_z_u[j], solution.z_u[j], rel_tol, abs_tol
            );
        }
    }

    return ok;
}
