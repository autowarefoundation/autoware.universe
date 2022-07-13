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
$begin subgraph_hes2jac.cpp$$
$spell
    Hessian
    Subgraphs
    Jacobian
$$

$section Sparse Hessian Using Subgraphs and Jacobian: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool subgraph_hes2jac(void)
{   bool ok = true;
    using CppAD::NearEqual;
    typedef CppAD::AD<double>                      a_double;
    typedef CPPAD_TESTVECTOR(double)               d_vector;
    typedef CPPAD_TESTVECTOR(a_double)             a_vector;
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
    // a_double version of x
    a_vector ax(n);
    for(size_t j = 0; j < n; j++)
        ax[j] = x[j];
    //
    // declare independent variables and starting recording
    CppAD::Independent(ax);
    //
    // a_double version of y = f(x) = 5 * x0 * x1 + sum_j xj^3
    size_t m = 1;
    a_vector ay(m);
    ay[0] = 5.0 * ax[0] * ax[1];
    for(size_t j = 0; j < n; j++)
        ay[0] += ax[j] * ax[j] * ax[j];
    //
    // create double version of f: x -> y and stop tape recording
    // (without executing zero order forward calculation)
    CppAD::ADFun<double> f;
    f.Dependent(ax, ay);
    //
    // Optimize this function to reduce future computations.
    // Perhaps only one optimization at the end would be faster.
    f.optimize();
    //
    // create a_double version of f
    CppAD::ADFun<a_double, double> af = f.base2ad();
    //
    // declare independent variables and start recording g(x) = f'(x)
    Independent(ax);
    //
    // Use one reverse mode pass to compute z = f'(x)
    a_vector aw(m), az(n);
    aw[0] = 1.0;
    af.Forward(0, ax);
    az = af.Reverse(1, aw);
    //
    // create double version of g : x -> f'(x)
    CppAD::ADFun<double> g;
    g.Dependent(ax, az);
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
// END C++
