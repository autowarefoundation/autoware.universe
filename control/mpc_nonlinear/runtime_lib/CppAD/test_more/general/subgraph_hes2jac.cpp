/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
Sparse Hessian Using Subgraphs and Jacobian: Example and Test
*/
# include <cppad/cppad.hpp>
bool subgraph_hes2jac(void)
{   bool ok = true;
    using CppAD::NearEqual;
    typedef CppAD::AD<double>                      a1double;
    typedef CppAD::AD<a1double>                    a2double;
    typedef CPPAD_TESTVECTOR(double)               d_vector;
    typedef CPPAD_TESTVECTOR(a1double)             a1vector;
    typedef CPPAD_TESTVECTOR(a2double)             a2vector;
    typedef CPPAD_TESTVECTOR(size_t)               s_vector;
    typedef CPPAD_TESTVECTOR(bool)                 b_vector;
    typedef CppAD::sparse_rcv<s_vector, d_vector>  sparse_matrix;
    //
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    //
    // double version of x
    size_t n = 12;
    d_vector x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = double(j + 2);
    //
    // a1double version of x
    a1vector a1x(n);
    for(size_t j = 0; j < n; j++)
        a1x[j] = x[j];
    //
    // a2double version of x
    a2vector a2x(n);
    for(size_t j = 0; j < n; j++)
        a2x[j] = a1x[j];
    //
    // declare independent variables and starting recording
    CppAD::Independent(a2x);
    //
    // a2double version of y = f(x) = 5 * x0 * x1 + sum_j xj^3
    size_t m = 1;
    a2vector a2y(m);
    a2y[0] = 5.0 * a2x[0] * a2x[1];
    for(size_t j = 0; j < n; j++)
        a2y[0] += a2x[j] * a2x[j] * a2x[j];
    //
    // create a1double version of f: x -> y and stop tape recording
    // (without executing zero order forward calculation)
    CppAD::ADFun<a1double> a1f;
    a1f.Dependent(a2x, a2y);
    //
    // Optimize this function to reduce future computations.
    // Perhaps only one optimization at the end would be faster.
    a1f.optimize();
    //
    // declare independent variables and start recording g(x) = f'(x)
    Independent(a1x);
    //
    // Use one reverse mode pass to compute z = f'(x)
    a1vector a1w(m), a1z(n);
    a1w[0] = 1.0;
    a1f.Forward(0, a1x);
    a1z = a1f.Reverse(1, a1w);
    //
    // create double version of g : x -> f'(x)
    CppAD::ADFun<double> g;
    g.Dependent(a1x, a1z);
    ok &= g.size_random() == 0;
    //
    // Optimize this function to reduce future computations.
    // Perhaps no optimization would be faster.
    g.optimize();
    //
    // compute f''(x) = g'(x)
    b_vector select_domain(n), select_range(n);
    for(size_t j = 0; j < n; ++j)
    {   select_domain[j] = true;
        select_range[j]  = true;
    }
    sparse_matrix hessian;
    g.subgraph_jac_rev(select_domain, select_range, x, hessian);
    // -------------------------------------------------------------------
    // check number of non-zeros in the Hessian
    // (only x0 * x1 generates off diagonal terms)
    ok &= hessian.nnz() == n + 2;
    //
    for(size_t k = 0; k < hessian.nnz(); ++k)
    {   size_t r = hessian.row()[k];
        size_t c = hessian.col()[k];
        double v = hessian.val()[k];
        //
        if( r == c )
        {   // a diagonal element
            double check = 6.0 * x[r];
            ok          &= NearEqual(v, check, eps, eps);
        }
        else
        {   // off diagonal element
            ok   &= (r == 0 && c == 1) || (r == 1 && c == 0);
            double check = 5.0;
            ok          &= NearEqual(v, check, eps, eps);
        }
    }
    ok &= g.size_random() > 0;
    g.clear_subgraph();
    ok &= g.size_random() == 0;
    return ok;
}
