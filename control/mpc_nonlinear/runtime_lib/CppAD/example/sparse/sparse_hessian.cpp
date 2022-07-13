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
$begin sparse_hessian.cpp$$
$spell
    Cpp
    Hessian
$$

$section Sparse Hessian: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool sparse_hessian(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    size_t i, j, k, ell;
    typedef CPPAD_TESTVECTOR(AD<double>)               a_vector;
    typedef CPPAD_TESTVECTOR(double)                     d_vector;
    typedef CPPAD_TESTVECTOR(size_t)                     i_vector;
    typedef CPPAD_TESTVECTOR(bool)                       b_vector;
    typedef CPPAD_TESTVECTOR(std::set<size_t>)         s_vector;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 12;  // must be greater than or equal 3; see n_sweep below
    a_vector a_x(n);
    for(j = 0; j < n; j++)
        a_x[j] = AD<double> (0);

    // declare independent variables and starting recording
    CppAD::Independent(a_x);

    // range space vector
    size_t m = 1;
    a_vector a_y(m);
    a_y[0] = a_x[0]*a_x[1];
    for(j = 0; j < n; j++)
        a_y[0] += a_x[j] * a_x[j] * a_x[j];

    // create f: x -> y and stop tape recording
    // (without executing zero order forward calculation)
    CppAD::ADFun<double> f;
    f.Dependent(a_x, a_y);

    // new value for the independent variable vector, and weighting vector
    d_vector w(m), x(n);
    for(j = 0; j < n; j++)
        x[j] = double(j);
    w[0] = 1.0;

    // vector used to check the value of the hessian
    d_vector check(n * n);
    for(ell = 0; ell < n * n; ell++)
        check[ell] = 0.0;
    ell        = 0 * n + 1;
    check[ell] = 1.0;
    ell        = 1 * n + 0;
    check[ell] = 1.0 ;
    for(j = 0; j < n; j++)
    {   ell = j * n + j;
        check[ell] = 6.0 * x[j];
    }

    // -------------------------------------------------------------------
    // second derivative of y[0] w.r.t x
    d_vector hes(n * n);
    hes = f.SparseHessian(x, w);
    for(ell = 0; ell < n * n; ell++)
        ok &=  NearEqual(w[0] * check[ell], hes[ell], eps, eps );

    // --------------------------------------------------------------------
    // example using vectors of bools to compute sparsity pattern for Hessian
    b_vector r_bool(n * n);
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            r_bool[i * n + j] = false;
        r_bool[i * n + i] = true;
    }
    f.ForSparseJac(n, r_bool);
    //
    b_vector s_bool(m);
    for(i = 0; i < m; i++)
        s_bool[i] = w[i] != 0;
    b_vector p_bool = f.RevSparseHes(n, s_bool);

    hes = f.SparseHessian(x, w, p_bool);
    for(ell = 0; ell < n * n; ell++)
        ok &=  NearEqual(w[0] * check[ell], hes[ell], eps, eps );

    // --------------------------------------------------------------------
    // example using vectors of sets to compute sparsity pattern for Hessian
    s_vector r_set(n);
    for(i = 0; i < n; i++)
        r_set[i].insert(i);
    f.ForSparseJac(n, r_set);
    //
    s_vector s_set(m);
    for(i = 0; i < m; i++)
        if( w[i] != 0. )
            s_set[0].insert(i);
    s_vector p_set = f.RevSparseHes(n, s_set);

    // example passing sparsity pattern to SparseHessian
    hes = f.SparseHessian(x, w, p_set);
    for(ell = 0; ell < n * n; ell++)
        ok &=  NearEqual(w[0] * check[ell], hes[ell], eps, eps );

    // --------------------------------------------------------------------
    // use row and column indices to specify upper triangle of
    // non-zero elements of Hessian
    size_t K = n + 1;
    i_vector row(K), col(K);
    hes.resize(K);
    k = 0;
    for(j = 0; j < n; j++)
    {   // diagonal of Hessian
        row[k] = j;
        col[k] = j;
        k++;
    }
    // only off diagonal non-zero elemenet in upper triangle
    row[k] = 0;
    col[k] = 1;
    k++;
    ok &= k == K;
    CppAD::sparse_hessian_work work;

    // can use p_set or p_bool.
    size_t n_sweep = f.SparseHessian(x, w, p_set, row, col, hes, work);
    for(k = 0; k < K; k++)
    {   ell = row[k] * n + col[k];
        ok &=  NearEqual(w[0] * check[ell], hes[k], eps, eps );
    }
    ok &= n_sweep == 2;

    // now recompute at a different x and w (using work from previous call
    w[0]       = 2.0;
    x[1]       = 0.5;
    ell        = 1 * n + 1;
    check[ell] = 6.0 * x[1];
    s_vector   not_used;
    n_sweep    = f.SparseHessian(x, w, not_used, row, col, hes, work);
    for(k = 0; k < K; k++)
    {   ell = row[k] * n + col[k];
        ok &=  NearEqual(w[0] * check[ell], hes[k], eps, eps );
    }
    ok &= n_sweep == 2;



    return ok;
}
// END C++
