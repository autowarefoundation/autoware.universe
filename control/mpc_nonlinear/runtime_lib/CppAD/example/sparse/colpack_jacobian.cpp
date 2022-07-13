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
$begin colpack_jacobian.cpp$$
$spell
    colpack_jac
    jacobian
$$

$section ColPack: Sparse Jacobian Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
bool colpack_jacobian(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    typedef CPPAD_TESTVECTOR(AD<double>) a_vector;
    typedef CPPAD_TESTVECTOR(double)     d_vector;
    typedef CppAD::vector<size_t>        i_vector;
    size_t i, j, k, ell;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 4;
    a_vector  a_x(n);
    for(j = 0; j < n; j++)
        a_x[j] = AD<double> (0);

    // declare independent variables and starting recording
    CppAD::Independent(a_x);

    size_t m = 3;
    a_vector  a_y(m);
    a_y[0] = a_x[0] + a_x[1];
    a_y[1] = a_x[2] + a_x[3];
    a_y[2] = a_x[0] + a_x[1] + a_x[2] + a_x[3] * a_x[3] / 2.;

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);

    // new value for the independent variable vector
    d_vector x(n);
    for(j = 0; j < n; j++)
        x[j] = double(j);

    /*
          [ 1 1 0 0  ]
    jac = [ 0 0 1 1  ]
          [ 1 1 1 x_3]
    */
    d_vector check(m * n);
    check[0] = 1.; check[1] = 1.; check[2]  = 0.; check[3]  = 0.;
    check[4] = 0.; check[5] = 0.; check[6]  = 1.; check[7]  = 1.;
    check[8] = 1.; check[9] = 1.; check[10] = 1.; check[11] = x[3];

    // Normally one would use f.ForSparseJac or f.RevSparseJac to compute
    // sparsity pattern, but for this example we extract it from check.
    std::vector< std::set<size_t> >  p(m);

    // using row and column indices to compute non-zero in rows 1 and 2
    i_vector row, col;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
        {   ell = i * n + j;
            if( check[ell] != 0. )
            {   row.push_back(i);
                col.push_back(j);
                p[i].insert(j);
            }
        }
    }
    size_t K = row.size();
    d_vector jac(K);

    // empty work structure
    CppAD::sparse_jacobian_work work;
    ok &= work.color_method == "cppad";

    // choose to use ColPack
    work.color_method = "colpack";

    // forward mode
    size_t n_sweep = f.SparseJacobianForward(x, p, row, col, jac, work);
    for(k = 0; k < K; k++)
    {   ell = row[k] * n + col[k];
        ok &= NearEqual(check[ell], jac[k], eps, eps);
    }
    ok &= n_sweep == 4;

    // reverse mode
    work.clear();
    work.color_method = "colpack";
    n_sweep = f.SparseJacobianReverse(x, p, row, col, jac, work);
    for(k = 0; k < K; k++)
    {   ell = row[k] * n + col[k];
        ok &= NearEqual(check[ell], jac[k], eps, eps);
    }
    ok &= n_sweep == 2;

    return ok;
}
// END C++
